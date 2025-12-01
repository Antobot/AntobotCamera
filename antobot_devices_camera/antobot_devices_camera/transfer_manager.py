import os
import re
import shutil
import subprocess
import threading
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node 
from rclpy.action import ActionServer 
from rclpy.executors import MultiThreadedExecutor

from antobot_camera_msgs.srv import (
    DeleteSessions,
    GetAvailability,
    GetBandwidth,
    GetSessionTransferList,
    StartTransfer,
)
from antobot_camera_msgs.msg import (
    SessionTransferItem,
    TransferStatus,
)

from antobot_devices_camera.wifi_transfer import WiFiTransfer
from antobot_devices_camera.usb_transfer import USBTransfer


class TransferManager(Node):
    """
    Orchestrates transferring sessions either over Wi-Fi (via INetwork)
    or to USB (via USBTransfer). ROS-specific pub/sub logic can live here.
    """

    def __init__(self):
        super().__init__('transfer_manager')

        # Load configuration
        self.declare_parameter('config_path', '/home/fubinzhang/Lab/ros2_zfb_ws/src/acCamera/antobot_devices_camera/config/scouting_config.yaml')
        # self.declare_parameter('config_path', '/home/scouting/ros2_ws/src/acCamera/antobot_devices_camera/config/scouting_config.yaml')
        self.cfg = self._load_cfg()

        # Unpack config details
        self.rec_dir = Path(self.cfg.get('transfer', {}).get('rec_dir'))
        self.log_dir = Path(self.cfg.get('transfer', {}).get('log_dir'))
        
        # WiFi transfer settings
        self.host = self.cfg.get("transfer", {}).get("SERVER_IP")
        port = 22
        self.username = self.cfg.get("transfer", {}).get("SERVER_USER")
        password = self.cfg.get("transfer", {}).get("SERVER_PWD",1234)
        self.remote_saving_path = self.cfg.get("transfer", {}).get("REMOTE_SAVING_PATH", f"/home/{self.username}")
        self.wifi_transfer = WiFiTransfer(self.host, port, self.username, password=password)
        self.robot_id = os.getenv('ROBOT_ID', "1")

        # USB transfer settings
        user = os.getenv('USER')
        if user:
            usb_dir = os.path.join('/media', user, 'ASCOPY')
        else:
            usb_dir = '/media/fubinzhang/ASCOPY'
        self.usb_transfer = USBTransfer(usb_dir)

        # Concurrency 
        self.usb_lock = threading.Lock()
        self.wifi_lock = threading.Lock()

        # States
        self.server_available = False
        self.usb_available = False

        # Transfer status tracking
        self.sessions = {}
        self.current_transfer_status = {
            'is_transferring': False,
            'destination': '',
            'progress': 0.0,
            'speed': 0,
            'time_remaining': 0,
            'bytes_remaining': 0,
        }

        ns = "/transfer_manager"
        # ==================== ROS2 Sevice ====================
        self.srv_get_tsfr_list = self.create_service(
            GetSessionTransferList,
            f"{ns}/get_session_list",
            self._service_callback_get_session_list
        )
        self.srv_delete_session = self.create_service(
            DeleteSessions,
            f"{ns}/delete_sessions",
            self._service_callback_delete_session
        )
        self.srv_get_availability = self.create_service(
            GetAvailability,
            f"{ns}/get_availability",
            self._service_callback_get_availability
        )
        self.srv_get_bandwidth = self.create_service(
            GetBandwidth,
            f"{ns}/get_bandwidth",
            self._service_callback_get_bandwidth
        )
        self.srv_start_transfer = self.create_service(
            StartTransfer,
            f"{ns}/start_transfer",
            self._service_callback_start_transfer
        )

        # ==================== ROS2 Publisher ====================
        self.transfer_status_pub = self.create_publisher(
            TransferStatus,
            f"{ns}/status",
            10
        )
        self.transfer_status_timer = None

    def publish_transfer_status(self):
        msg = TransferStatus()
        msg.is_transferring = self.current_transfer_status['is_transferring']
        msg.destination = self.current_transfer_status['destination']
        msg.progress = float(self.current_transfer_status['progress'])
        msg.speed = float(self.current_transfer_status['speed'])
        msg.time_remaining = int(self.current_transfer_status['time_remaining'])
        msg.bytes_remaining = int(self.current_transfer_status['bytes_remaining'])
        self.transfer_status_pub.publish(msg)

    def _load_cfg(self):
        path = self.get_parameter('config_path').get_parameter_value().string_value
        self.get_logger().info(f"Loading config from: {path}")
        with open(path, 'r') as f:
            return yaml.safe_load(f) or {}

    ###########################################################################################################
    ## Callbacks                                                                
    ###########################################################################################################

    def _service_callback_get_session_list(self, request, response):
        """
        scan the recording directory for sessions,
        read state indicators from manifest.yaml,
        if marked for upload, save sessions to a list
        """

        response.session_list = []

        self.sessions.clear()
        # iterate through each item in the recording directory
        for entry in self.rec_dir.iterdir():
            # only consider directories, ignore files
            if not entry.is_dir():
                continue         

            manifest = entry / "manifest.yaml"       
            try:
                # open manifest
                data = yaml.safe_load(manifest.read_text())
                if not (data.get("records")):
                    continue

                current_session = SessionTransferItem(
                    session_name=entry.name,
                    session_time=data["session_start_timestamp"],
                    description=data["user_description"],
                    upload_complete=data["upload_complete"],
                    copy_complete=data["usb_transfer_complete"],
                )

                for f in entry.iterdir():
                    if f.is_file():
                        current_session.file_list.append(f.name)
                        current_session.size += f.stat().st_size
                        current_session.num_files += 1
                        ext = f.suffix
                        if ext not in current_session.file_types:
                            current_session.file_types.append(ext)
                response.session_list.append(current_session)
                self.sessions[current_session.session_name] = current_session      
            except FileNotFoundError as err:
                # if the file isn't found, look at next session dir
                # any other exception can be raised
                print(f"File not found: {err}")
                continue
            except Exception as e:
                print(f"Error when read {manifest}: {e}")
        return response


    def _service_callback_delete_session(self, request, response):

        # Don’t allow deletes while a transfer is running
        if self.current_transfer_status["is_transferring"]:
            response.success = False
            response.message = "Transfer in progress – delete rejected"
            return response

        failed = []                                     
        with self.usb_lock and self.wifi_lock:         # avoid racing USB/WIFI ops
            for sess in request.session_list:
                path = self.rec_dir / sess.session_name
                try:
                    if path.exists():
                        shutil.rmtree(path)             # may raise OSError
                    else:
                        failed.append(sess.session_name)
                except Exception as e:
                    self.get_logger().error(f"Failed to delete {path}: {e}")
                    failed.append(sess.session_name)
        # Build the response
        if not failed:
            response.success = True
            response.message = "All sessions deleted"
        else:
            response.success = False
            response.message = f"Failed to delete: {', '.join(failed)}"
        return response


    def _service_callback_get_availability(self, request, response):

        destination = request.destination
        if destination == 'usb':
            with self.usb_lock:
                self.usb_available = self.usb_transfer.is_usb_connected()

                response.available = self.usb_available
                response.message = f"USB is {'available' if self.usb_available else 'unavailable'}."

                self.get_logger().info(response.message)

        elif destination == 'server':
            with self.wifi_lock:
                self.server_available = self.wifi_transfer.check_wifi_upload_availability()
                
                response.available = self.server_available
                response.message = f"Server is {'available' if self.server_available else 'unavailable'}."

                self.get_logger().info(response.message)
        
        return response


    def _service_callback_get_bandwidth(self, request, response):
        destination = request.destination
        response.bandwidth = 0.0
        
        if destination == 'usb':
            if self.current_transfer_status["is_transferring"] or (not self.usb_lock.acquire()):
                response.bandwidth = self.current_transfer_status["speed"]
                return response
            try:
                response.bandwidth = self.usb_transfer.get_copy_bandwidth()
            finally:
                self.usb_lock.release()
        elif destination == 'server':
            if self.current_transfer_status["is_transferring"] or (not self.wifi_lock.acquire()):
                response.bandwidth = self.current_transfer_status["speed"]
                return response
            try:    
                response.bandwidth = self.wifi_transfer.get_upload_bandwidth(self.remote_saving_path)
            finally:
                self.wifi_lock.release()

        return response
            
    def _service_callback_start_transfer(self, request, response):
        """Start transfer service callback.(USB or WIFI)"""

        if self.current_transfer_status["is_transferring"]:
            response.success = False
            response.message = "Another transfer running"
            return response
        
        self.current_transfer_status.update(
            is_transferring=True,
            destination=request.destination,
            progress=0.0,
            speed=0,
            time_remaining=0,
            bytes_remaining=0,
        )
        self.transfer_status_timer = self.create_timer(1.0, self.publish_transfer_status)

        threading.Thread(
            target=self.transfer_files,
            args=(request,),
            daemon=True
        ).start()
        while True:
            if self.current_transfer_status["is_transferring"] and self.current_transfer_status["progress"] > 0:
                response.success = True
                response.message = f"Transfer is starting"
                return response
    

    def transfer_files(self, request):
        """Background thread to perform the transfer."""

        destination = request.destination
        transfer_list = request.transfer_list

        failures = []
        total = sum(s.size for s in transfer_list)

        if destination == 'usb':
            with self.usb_lock:
                for s in transfer_list:
                    src = self.rec_dir / s.session_name
                    if self.robot_id:
                        dst = Path(self.usb_transfer.usb_dir) / s.session_name / self.robot_id
                    else:
                        dst = Path(self.usb_transfer.usb_dir) / s.session_name
                    dst.mkdir(parents=True, exist_ok=True)
                    print(f"Starting transfer of {src} to {dst}")
                    self.get_logger().info(f"Uploading {s.session_name} to {dst}...")
                    try:
                        pct = self.transfer_session_with_feedback(src, dst)
                        # update manifest.yaml status
                        self.usb_transfer.update_upload_state(src, robot_id=self.robot_id)
                        self.get_logger().info(f"{s.session_name} copy complete ({pct:.0f}%)")
                    except Exception as e:
                        self.get_logger().error(f"copy failed for {s.session_name}: {e}")
                        failures.append(s.session_name)
            self.usb_transfer.unmount()
            success = not failures
            message = "All files copied" if not failures else f"Copy failed: {', '.join(failures)}"
        
        elif destination == 'server':
            with self.wifi_lock:
                for s in transfer_list:
                    src = self.rec_dir / s.session_name
                    if self.robot_id:
                        dst = f"{self.username}@{self.host}:{os.path.join(self.remote_saving_path, s.session_name, self.robot_id)}"
                        # Ensure the remote directory exists
                        try:
                            subprocess.run(
                                ["ssh", f"{self.username}@{self.host}", "mkdir", "-p", dst.split(":", 1)[1]],
                                check=True
                            )
                        except subprocess.CalledProcessError as e:
                            self.get_logger().error(f"Failed to create remote directory {dst}: {e}")
                    else:
                        dst = f"{self.username}@{self.host}:{os.path.join(self.remote_saving_path, s.session_name)}"

                    self.get_logger().info(f"Uploading {s.session_name} to {dst}...")
                    try:
                        # Transfer session
                        pct = self.transfer_session_with_feedback(src, dst)

                        # Update manifest.yaml state (both local and remote)
                        self.wifi_transfer.update_upload_state(src, robot_id=self.robot_id, remote_path=self.remote_saving_path)

                        self.get_logger().info(f"{s.session_name} upload complete ({pct:.0f}%)")
                    except Exception as e:
                        self.get_logger().error(f"Upload failed for {s.session_name}: {e}")
                        failures.append(s.session_name)

                success = not failures
                message = "All files uploaded successfully." if success else f"Upload failed: {', '.join(failures)}"

        if self.transfer_status_timer is not None:
            self.transfer_status_timer.cancel()
            self.transfer_status_timer = None

        self.current_transfer_status.update(
            is_transferring=False,
            destination='',
            progress=100.0 if success else 0.0,
            speed=0,
            time_remaining=0,
            bytes_remaining=0,
        )

        self.publish_transfer_status()
        return success, message

    ###########################################################################################################
    def transfer_session_with_feedback(self, src, dst):
        """
        Upload a folder using rsync, parse progress, and publish ROS feedback.
        """

        src_str = os.fspath(src)          # Path  -> str  or str stays str
        if not src_str.endswith('/'):     # ensure trailing slash for rsync
            src_str += '/'
        
        dst_str = os.fspath(dst) 
        
        cmd = [
            "rsync",
            "-ahvv",
            "--no-group",
            "--no-owner",
            "--info=progress2",
            "--no-inc-recursive",
            "--no-compress",
            src_str, dst_str
        ]

        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        
        percent = 0.0
        for line in iter(proc.stdout.readline, ''):
            line = line.strip()
            match = re.match(r"([\d.]+[KMG]?)\s+(\d+)%\s+([\d.]+[A-Z]?B/s)\s+(\d+:\d+:\d+)", line)
            if match:
                bytes_transferred_str = match.group(1)
                percent = float(match.group(2))
                speed_str = match.group(3)
                time_remain_str = match.group(4)

                bytes_transferred = self._parse_size_to_bytes(bytes_transferred_str)
                time_remaining = self._hms_to_seconds(time_remain_str)

                speed = self._parse_speed_to_mb(speed_str)
                if percent > 0:
                    total_bytes = int(bytes_transferred * 100 / percent)
                    bytes_remaining = total_bytes - bytes_transferred
                else:
                    bytes_remaining = 0
                
                self.current_transfer_status.update({
                    'progress': percent,
                    'speed': speed,
                    'time_remaining': time_remaining,
                    'bytes_remaining': bytes_remaining,
                })


        proc.wait()
        if proc.returncode != 0:
            raise RuntimeError(f"rsync failed with return code {proc.returncode}")

        if os.fspath(dst).startswith(self.usb_transfer.usb_dir):
            os.sync()   # flush all buffers to the USB

        return percent


    def _hms_to_seconds(self, hms):
        h, m, s = map(int, hms.split(":"))
        return h * 3600 + m * 60 + s


    def _parse_speed_to_mb(self, speed_str):
        """Convert rsync speed string (e.g., '52.08MB/s') to MB/s as float."""
        speed_str = speed_str.strip()
        
        import re
        match = re.match(r"([\d.]+)([KMG]?)B/s", speed_str)
        if not match:
            return 0.0
        number = float(match.group(1))
        suffix = match.group(2)
        
        if suffix == 'K':
            return number / 1024  # KB/s to MB/s
        elif suffix == 'M':
            return number         # Already MB/s
        elif suffix == 'G':
            return number * 1024  # GB/s to MB/s
        else:
            return number / (1024 * 1024)  # B/s to MB/s
    
    def _parse_size_to_bytes(self, size_str):
        """Convert size string like '218.37M' to bytes"""
        if size_str.endswith('K'):
            return int(float(size_str[:-1]) * 1024)
        elif size_str.endswith('M'):
            return int(float(size_str[:-1]) * 1024 * 1024)
        elif size_str.endswith('G'):
            return int(float(size_str[:-1]) * 1024 * 1024 * 1024)
        else:
            return int(float(size_str))     # Assume bytes if no suffix
    


def main(args=None):
    rclpy.init(args=args)
    transfer_mgr = TransferManager()
    executor = MultiThreadedExecutor()
    rclpy.spin(transfer_mgr, executor=executor)
    transfer_mgr.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()