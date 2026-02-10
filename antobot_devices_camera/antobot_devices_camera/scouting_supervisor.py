import threading
import time
import json
import yaml
from threading import Lock

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from antobot_camera_msgs.srv import ScoutingCommand
from antobot_devices_camera.realsense_camera import CameraDriver
from antobot_devices_camera.camera_record_manager import CameraRecordManager
from antobot_devices_camera.preview_streamer import PreviewStreamer, CameraStreamTrack
from antobot_devices_camera.transfer_manager import TransferManager

class ScoutingSupervisor(Node):
    def __init__(self, config_path):
        super().__init__('scouting_supervisor')
        
        # Load Config
        with open(config_path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        # Scouting Status Publisher
        self.scouting_status_pub = self.create_publisher(String, '/scouting/system_status', 1)  
        self.scouting_status_timer = self.create_timer(1.0, self._publish_system_status)

        # Initialize Camera Drivers
        self.camera_drivers = {}
        self._init_camera()
        
        # Initialize Record Manager
        self.record_mgr = CameraRecordManager(self.cfg, self.get_logger())
        # Initialize Transfer Manager
        self.transfer_pub = self.create_publisher(String, '/scouting/transfer_status', 1)
        self.transfer_mgr = TransferManager(self.cfg, logger=self.get_logger(), progress_callback=self._transfer_cb)
        self.is_transferring = False
        # Initialize Preview Streamer 
        self._init_preview()

        # GPS State
        self.gps_lock = Lock()
        self.gps_log = []
        self.is_gps_logging = False
        self._setup_gps_sub()

        # Service Setup
        self.srv_group = ReentrantCallbackGroup()
        self.srv = self.create_service(ScoutingCommand, "/scouting/command", self._command_srv_cb, callback_group=self.srv_group)
        
        # Frame Distribution Threads, one per camera
        self.running = True
        self.pump_threads = []
        for loc in self.camera_drivers:
            t = threading.Thread(target=self._distribute_frames, args=(loc,), daemon=True)
            t.start()
            self.pump_threads.append(t)

        self.get_logger().info("Scouting Supervisor Ready. Service: /scouting/command")


    def _init_camera(self):
        cameras = self.cfg.get('camera', {})
        for loc, conf in cameras.items():
            try:
                params = conf['params']
                d = CameraDriver(port=conf.get('port_core'), width=params.get('width', 1280), height=params.get('height', 720), fps=params.get('framerate', 30))
                self.camera_drivers[loc] = d
            except Exception as e:
                self.get_logger().error(f"Failed to init {loc}: {e}")


    def _setup_gps_sub(self):
        if t := self.cfg.get('gps_topic'): self.create_subscription(NavSatFix, t, self._gps_cb, qos_profile_sensor_data)
        

    def _gps_cb(self, msg):
        if self.is_gps_logging:
            with self.gps_lock:
                self.gps_log.append({"t": msg.header.stamp.sec, "lat": msg.latitude, "lon": msg.longitude})

    def _init_preview(self):
        tracks = {loc: CameraStreamTrack(d.read_request_array, (d.height, d.width)) for loc, d in self.camera_drivers.items()}
        self.streamer = PreviewStreamer(tracks)
        threading.Thread(target=self.streamer.run, daemon=True).start()
        

    def _distribute_frames(self, loc):
        '''
        Central loop that pulls frames from the camera driver, updates preview,
        and pushes frames to the record manager and inference manager.
        '''
        driver = self.camera_drivers[loc]
        while self.running:
            if not getattr(driver, '_running', False):
                time.sleep(0.5) 
                continue

            try:
                # This loop updates 'last_color_frame' inside the driver
                for color, depth in driver.frames():
                    if not self.running: break
                    
                    # 1. Update Preview (Implicit: driver.last_color_frame is updated)
                    
                    # 2. Push to Recorder, record manager side will refuse the frame if not recording 
                    self.record_mgr.handle_frame(loc, color, depth)

                    # 3. Push to Inference, inference manager side will refuse if not active
                    # self.inference_mgr.push_frame(loc, color)
                    
            except Exception as e:
                # If the driver is flagged as not running (because we just stopped it),
                # this error is expected. We suppress the log.
                if not getattr(driver, '_running', False):
                    self.get_logger().info(f"Pump {loc} stopped gracefully (Hardware disabled).")
                else:
                    # Only log if it was supposed to be running
                    self.get_logger().error(f"Pump {loc} failed unexpectedly: {e}")
                time.sleep(1) 

    def _transfer_cb(self, status_dict):
            """
            Updates local state flag AND publishes detailed progress
            """
            # 1. Update local flag for the Heartbeat
            s = status_dict.get('status')
            if s == 'running':
                self.is_transferring = True
            elif s in ['finished', 'error', 'stopped']:
                self.is_transferring = False

            # 2. Publish detailed progress (for the progress bar)
            # We keep this for the high-frequency updates needed by the specific Transfer Screen
            msg = String()
            msg.data = json.dumps({"type": "transfer_progress", "data": status_dict})
            self.transfer_pub.publish(msg, )
    
    def _publish_system_status(self):
        """
        Aggregates state from all managers into one JSON packet.
        Json example:   
        {
            "hardware": {
                "left_camera": true,
                "right_camera": false
            },
            "tasks": {
                "preview": true,
                "recording": false,
                "transfer": true,
                "inference": false
            }
        }
        """
        # A. Hardware Status (Drivers)
        hw_status = {}
        any_hardware_on = False
        for loc, driver in self.camera_drivers.items():
            is_on = getattr(driver, '_running', False)
            hw_status[loc] = is_on
            if is_on: any_hardware_on = True

        # B. Recording Status
        rec_active_map = getattr(self.record_mgr, 'recording_active', {})
        is_recording = any(rec_active_map.values())

        # C. Construct Payload
        payload = {
            "hardware": hw_status,
            "tasks": {
                "preview": any_hardware_on,  # If hardware is on, preview is available, its actually status depends on streamer clients
                "recording": is_recording,
                "transfer": self.is_transferring,
                "inference": False  # Placeholder for future
            }
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.scouting_status_pub.publish(msg)

    # --- SERVICE CALLBACK ---
    def _command_srv_cb(self, request, response):
        try:
            cmd = json.loads(request.command_json)
            task = cmd.get('task')
            op = cmd.get('op')
            params = cmd.get('params', {})
            
            self.get_logger().info(f"Service Request: Task={task}, Op={op}")
            
            result_data = {"status": "error", "msg": "Unknown task", "details": {}}

            if task == "recording":
                result_data = self._handle_recording(op, params)
            elif task == "transfer":
                result_data = self._handle_transfer(op, params)
            elif task == "preview":
                result_data = self._handle_preview(op, params)
            
            # The Service "success" bool is strictly for "Did the command parse correctly?"
            # Domain errors (like "Camera not found") are returned inside the JSON status.
            response.success = True 
            response.response_json = json.dumps(result_data)
            
        except json.JSONDecodeError:
            response.success = False
            response.response_json = json.dumps({"status": "fatal", "msg": "Invalid JSON"})
        except Exception as e:
            self.get_logger().error(f"Service Exception: {e}")
            response.success = False
            response.response_json = json.dumps({"status": "fatal", "msg": str(e)})

        return response

    def _handle_recording(self, op, params):
        # Use available drivers if 'cameras' list is not provided
        targets = params.get('cameras') or list(self.camera_drivers.keys())
        details = {}
        if op == "start":
            success_count = 0
            for loc in targets:
                if loc not in self.camera_drivers: 
                    details[loc] = {"success": False, "msg": "No Hardware"}
                    continue
                d = self.camera_drivers[loc]
                if not getattr(d, '_running', False):
                    try: d.start()
                    except Exception as e: 
                        details[loc] = {"success": False, "msg": f"Start Fail: {e}"}
                        continue
                
                ret = self.record_mgr.start_recording(loc, d)
                ok, out = ret
                details[loc] = {"success": ok, "msg": out}
                if ok: success_count += 1

            if success_count > 0 and not self.is_gps_logging:
                with self.gps_lock: self.is_gps_logging = True; self.gps_log.clear()
            
            status = "ok" if success_count == len(targets) else ("partial" if success_count > 0 else "error")
            return {"status": status, "msg": f"Started {success_count}/{len(targets)}", "details": details}

        elif op == "stop":
            stop_hw = params.get('stop_hardware', True)
            for loc in targets:
                ret = self.record_mgr.stop_recording(loc)
                if not isinstance(ret, tuple) or len(ret) != 2:
                     details[loc] = {"success": False, "msg": "Internal Stop Error"}
                     continue
                ok, msg = ret
                details[loc] = {"success": ok, "msg": msg}
                
                if stop_hw and loc in self.camera_drivers: 
                    try: self.camera_drivers[loc].stop()
                    except: pass
            
            if not any(self.record_mgr.recording_active.values()) and self.is_gps_logging:
                with self.gps_lock: self.is_gps_logging = False
            return {"status": "ok", "msg": "Stopped", "details": details}
        return {"status": "error", "msg": "Invalid Op"}

    def _handle_transfer(self, op, params):
        # 1. LIST available sessions
        if op == "list":
            sessions = self.transfer_mgr.get_session_list()
            return {"status": "ok", "msg": f"Found {len(sessions)}", "details": {"sessions": sessions}}
            
        # 2. INFO (Hardware availability)
        elif op == "info":
            avail = self.transfer_mgr.get_availability()
            return {"status": "ok", "msg": "Status retrieved", "details": avail}

        # 3. START Transfer (takes destination and session list)
        elif op == "start":
            dest = params.get('destination')
            sessions = params.get('sessions', [])
            success, msg = self.transfer_mgr.start_transfer(dest, sessions)
            return {"status": "ok" if success else "error", "msg": msg, "details": {}}
            
        # 4. STOP Transfer
        elif op == "stop":
            success, msg = self.transfer_mgr.stop_transfer()
            return {"status": "ok" if success else "error", "msg": msg, "details": {}}

        # 5. DELETE Sessions
        elif op == "delete":
            sessions = params.get('sessions', [])
            success, msg = self.transfer_mgr.delete_sessions(sessions)
            return {"status": "ok" if success else "error", "msg": msg, "details": {}}

        return {"status": "error", "msg": "Invalid Transfer Op"}

    def _handle_preview(self, op, params):
        """
        Manages Hardware for Preview Mode.
        """
        # Use available drivers if 'cameras' list is not provided
        targets = params.get('cameras') or list(self.camera_drivers.keys())
        details = {}
        if op == "start":
            for loc in targets:
                if loc not in self.camera_drivers: continue
                # check if recording is active
                if self.record_mgr.recording_active.get(loc, False):
                    details[loc] = "Ignored (Recording Active)"
                    continue
                d = self.camera_drivers[loc]
                try:
                    # 1. Start Hardware
                    if not getattr(d, '_running', False): 
                        d.start()
                        details[loc] = "Hardware Started"
                    else:
                        details[loc] = "Hardware Already Running"
                except Exception as e:
                    details[loc] = f"Error: {e}"
            return {"status": "ok", "msg": "Preview Started", "details": details}

        elif op == "stop":
            for loc in targets:
                if loc not in self.camera_drivers: continue
                
                # SAFETY CHECK: If Recording is active, DO NOT stop hardware
                if self.record_mgr.recording_active.get(loc, False):
                    details[loc] = "Ignored (Recording Active)"
                    continue

                d = self.camera_drivers[loc]
                try:
                    # 1. Stop Hardware
                    if getattr(d, '_running', False):
                        d.stop()
                        details[loc] = "Hardware Stopped"
                    else:
                        details[loc] = "Already Stopped"
                except Exception as e:
                    details[loc] = f"Error: {e}"
            return {"status": "ok", "msg": "Preview Stopped", "details": details}

        elif op == "apply_config":
            res = {loc: d.apply_config(params.get('settings', {})) for loc, d in self.camera_drivers.items()}
            return {"status": "ok", "msg": "Config Applied", "details": res}
            
        return {"status": "error", "msg": "Invalid Op"}

    def destroy_node(self):
        self.running = False
        for d in self.camera_drivers.values(): d.stop()
        super().destroy_node()