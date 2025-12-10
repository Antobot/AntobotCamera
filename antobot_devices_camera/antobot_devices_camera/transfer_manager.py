import os
import re
import shutil
import subprocess
import threading
from pathlib import Path
import logging

from antobot_devices_camera.wifi_transfer import WiFiTransfer
from antobot_devices_camera.usb_transfer import USBTransfer

class TransferManager:
    """
    Orchestrates transferring sessions via Wi-Fi or USB.
    Managed by ScoutingSupervisor.
    """

    def __init__(self, config, logger=None, progress_callback=None):
        self.cfg = config
        self.logger = logger or logging.getLogger("TransferManager")
        self.progress_callback = progress_callback # Function(status_dict)

        # Unpack config details
        self.rec_dir = Path(self.cfg.get('recording_path', '/root/ros2_ws/data/'))
        
        # WiFi transfer settings
        self.host = self.cfg.get("transfer", {}).get("SERVER_IP")
        port = 22
        self.username = self.cfg.get("transfer", {}).get("SERVER_USER")
        password = self.cfg.get("transfer", {}).get("SERVER_PWD", 1234)
        self.remote_saving_path = self.cfg.get("transfer", {}).get("REMOTE_SAVING_PATH", f"/home/{self.username}")
        self.wifi_transfer = WiFiTransfer(self.host, port, self.username, password=password)
        self.robot_id = str(self.cfg.get('ROBOT_ID', None))

        # USB transfer settings
        user = os.getenv('USER', 'root')
        # Default to a safe path if not found
        usb_base = f'/media/{user}/ASCOPY'
        self.usb_transfer = USBTransfer(usb_base)

        # Concurrency 
        self.usb_lock = threading.Lock()
        self.wifi_lock = threading.Lock()

        # State tracking
        self.current_transfer_status = {
            'status': 'idle',
            'is_transferring': False,
            'destination': '',
            'progress': 0.0,
            'speed': '0.00MB/s', # Default value
            'eta': '0:00:00',    # Default value
            'current_file': ''
        }
        
        self.stop_event = threading.Event()
        self.transfer_thread = None

    # --- Public API called by Supervisor ---

    def get_session_list(self):
        """
        Scan recording directory for sessions.
        Returns list of dicts.
        """
        session_list = []
        self.logger.info(f"Scanning for sessions in {self.rec_dir}")
        if not self.rec_dir.exists():
            return session_list

        for entry in self.rec_dir.iterdir():
            if not entry.is_dir(): continue

            try:
                item = {
                    "session_name": entry.name,
                    "size": sum(f.stat().st_size for f in entry.iterdir() if f.is_file()),
                    "file_count": len([f for f in entry.iterdir() if f.is_file()])
                }
                session_list.append(item)
            except Exception as e:
                self.logger.error(f"Error scanning {entry}: {e}")
        return session_list

    def delete_sessions(self, session_names):
        if self.current_transfer_status["is_transferring"]:
            return False, "Transfer in progress"

        failed = []
        with self.usb_lock and self.wifi_lock: # Reuse locks just to be safe
            for name in session_names:
                path = self.rec_dir / name
                try:
                    if path.exists():
                        shutil.rmtree(path)
                    else:
                        failed.append(name)
                except Exception as e:
                    self.logger.error(f"Failed to delete {path}: {e}")
                    failed.append(name)
        
        if failed:
            return False, f"Failed: {failed}"
        return True, "Deleted"

    def get_availability(self):
        """Returns dict of {usb: bool, wifi: bool}"""
        # Note: This might block briefly
        usb = self.usb_transfer.is_usb_connected()
        wifi = self.wifi_transfer.check_wifi_upload_availability()
        return {"usb": usb, "wifi": wifi}

    def get_bandwidth(self, destination):
        """
        Returns estimated bandwidth in MB/s.
        If transferring, returns current speed.
        If idle, queries the interface.
        """
        if self.current_transfer_status["is_transferring"]:
            # Parse the string (e.g., "52.08MB/s") into a float
            return self._parse_speed_to_mb(self.current_transfer_status.get('speed', '0MB/s'))

        if destination == 'usb':
            if not self.usb_lock.acquire(blocking=False): return 0.0
            try:
                return self.usb_transfer.get_copy_bandwidth()
            except: return 0.0
            finally: self.usb_lock.release()
            
        elif destination == 'server':
            if not self.wifi_lock.acquire(blocking=False): return 0.0
            try:
                return self.wifi_transfer.get_upload_bandwidth(self.remote_saving_path)
            except: return 0.0
            finally: self.wifi_lock.release()
            
        return 0.0

    def start_transfer(self, destination, session_names):
        if self.current_transfer_status["is_transferring"]:
            return False, "Already transferring"

        self.stop_event.clear()
        
        # Reset Status
        self.current_transfer_status.update({
            'status': 'running',
            'is_transferring': True,
            'destination': destination,
            'progress': 0.0,
            'speed': 'Calculating...',
            'eta': '--:--:--'
        })

        self.transfer_thread = threading.Thread(
            target=self._transfer_worker,
            args=(destination, session_names),
            daemon=True
        )
        self.transfer_thread.start()
        return True, "Transfer started"

    def stop_transfer(self):
        if self.transfer_thread and self.transfer_thread.is_alive():
            self.stop_event.set()
            return True, "Stopping..."
        return False, "Not running"

    # --- Internal Worker ---

    def _transfer_worker(self, destination, session_names):
        self.logger.info(f"Starting transfer to {destination}")
        failures = []
        global_error = None # Track catastrophic errors

        try:
            for session_name in session_names:
                if self.stop_event.is_set(): break
                
                src = self.rec_dir / session_name
                self.current_transfer_status['current_file'] = session_name
                
                try:
                    if destination == 'usb':
                        with self.usb_lock:
                            remote_path = Path(self.usb_transfer.usb_dir) / session_name
                            if self.robot_id:
                                remote_path = os.path.join(remote_path, self.robot_id)
                            remote_path.mkdir(parents=True, exist_ok=True)
                            self._rsync_transfer(src, remote_path)
                            
                    elif destination == 'server':
                        with self.wifi_lock:
                            remote_path = f"{self.username}@{self.host}:{self.remote_saving_path}/{session_name}"
                            if self.robot_id:
                                remote_path = os.path.join(remote_path, self.robot_id)
                            
	                        # Ensure the remote directory exists
                            try:
                                subprocess.run(
                                    ["ssh", f"{self.username}@{self.host}", "mkdir", "-p", remote_path.split(":", 1)[1]],
                                    check=True
                                )
                            except subprocess.CalledProcessError as e:
                                self.logger.error(f"Failed to create remote directory {remote_path}: {e}")

                            self.logger.info(f'remote_path, {remote_path}')
                            self._rsync_transfer(src, remote_path)

                except Exception as e:
                    self.logger.error(f"Failed {session_name}: {e}")
                    failures.append(session_name)
                    # Stop completely on critical rsync errors? 
                    # Uncomment next line if you want to stop on first error
                    # global_error = str(e); break 

        except Exception as e:
            self.logger.error(f"Worker crash: {e}")
            global_error = str(e)
        
        finally:
            if destination == 'usb':
                self.usb_transfer.unmount()
            
            # DETERMINISTIC STATUS LOGIC
            if global_error:
                final_status = 'error'
                msg = global_error
            elif failures:
                final_status = 'error'
                msg = f"Failed sessions: {', '.join(failures)}"
            elif self.stop_event.is_set():
                final_status = 'stopped'
                msg = "User stopped transfer"
            else:
                final_status = 'finished'
                msg = ""

            self.current_transfer_status.update({
                'is_transferring': False,
                'status': final_status,
                'progress': 100.0 if final_status == 'finished' else 0.0,
                'speed': 'Done',
                'eta': '0:00:00',
                'error_msg': msg
            })
            
            if self.progress_callback:
                self.progress_callback(self.current_transfer_status)


    def _rsync_transfer(self, src, dst):
        import traceback
        from collections import deque
        """
        Executes rsync and parses progress for the GUI/Callback.
        includes thread-safe error handling and robust regex parsing.
        """
        self.logger.info(f'_rsync_transfer started')
        
        try:
            src_str = os.fspath(src)
            if not src_str.endswith('/'): src_str += '/'
            dst_str = os.fspath(dst)
            
            # -vv is very verbose (prints every file). 
            # --info=progress2 provides the total transfer progress.
            cmd = [
                "rsync", "-ahvv", "--no-group", "--no-owner", 
                "--info=progress2", "--no-inc-recursive", "--no-compress",
                src_str, dst_str
            ]

            proc = subprocess.Popen(
                cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT, 
                text=True, 
                bufsize=1
            )
            
            self.logger.info('-------------------')
            
            # --- REGEX EXPLANATION ---
            # 1. ([\d.]+)   -> Capture the number (e.g., "12.5") -> Group 1
            # 2. ([KMG]?)   -> Capture the unit (e.g., "M")      -> Group 2
            # 3. (\d+)%     -> Capture the percent integer       -> Group 3
            # 4. ([\d.]+)   -> Capture speed number              -> Group 4
            # 5. ([A-Z]?B/s)-> Capture speed unit                -> Group 5
            # 6. (\d+:\d+)  -> Capture ETA                       -> Group 6
            progress_re = re.compile(r"([\d.]+)([KMG]?)\s+(\d+)%\s+([\d.]+)([A-Z]?B/s)\s+(\d+:\d+:\d+)")
            
            # Keep track of the last 5 lines. If rsync fails, the error is usually here.
            recent_output = deque(maxlen=5)

            for line in iter(proc.stdout.readline, ''):
                if self.stop_event.is_set():
                    self.logger.info("Stop event set, terminating rsync.")
                    proc.terminate()
                    break
                    
                line = line.strip()
                if not line: continue
                
                recent_output.append(line)
                
                # Print to standard out for debugging (bypasses logger issues)
                # Once it works, you can comment this out to reduce noise
                # print(f"RSYNC: {line}") 

                match = progress_re.search(line)
                if match:
                    try:
                        # Update status
                        # Note: float() is now safe because Group 1 is numbers only.
                        transferred_amount = float(match.group(1))
                        percentage = float(match.group(3)) 
                        
                        self.current_transfer_status.update({
                            'transferred': transferred_amount,
                            'unit': match.group(2),
                            'progress': percentage, # Actual % (0-100)
                            'speed': f"{match.group(4)}{match.group(5)}",
                            'eta': match.group(6)
                        })
                        
                        if self.progress_callback:
                            self.progress_callback(self.current_transfer_status)
                            
                    except ValueError as e:
                        self.logger.error(f"Float conversion failed on line: {line} | Error: {e}")
                else:
                    # If it doesn't match the progress bar, it's a filename or error.
                    if "error" in line.lower() or "failed" in line.lower():
                        self.logger.error(f"Rsync Error Line: {line}")

            proc.wait()
            
            # Check return code
            if proc.returncode != 0 and not self.stop_event.is_set():
                error_log = "\n".join(recent_output)
                raise RuntimeError(f"Rsync Failed (Code {proc.returncode}). Last output:\n{error_log}")

        except Exception as e:
            # CRITICAL: This catches the silent thread death and prints it to the console
            print("!!! EXCEPTION IN RSYNC THREAD !!!")
            traceback.print_exc()
            self.logger.exception("Exception in _rsync_transfer")
            raise e # Re-raise to ensure the caller knows it failed


