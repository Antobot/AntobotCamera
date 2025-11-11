import os
import json
import yaml
import datetime
from typing import Dict, List
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix

from antobot_camera_msgs.srv import CameraRecord as CameraRecordSrv
from antobot_devices_camera.realsense_camera import CameraDriver
from antobot_devices_camera.recorder import Recorder


# camera_num: 3=left, 4=right, 0=both
NUM_TO_LOC = {3: "left", 4: "right"}

class CameraRecordManager(Node):
    def __init__(self):
        super().__init__('camera_record_manager')
        self.declare_parameter('config_path', '/root/ros2_ws/src/acCamera/antobot_devices_camera/config/scouting_config.yaml')
        self.cfg = self._load_cfg()

        self.recorders: Dict[str, Recorder] = {}
        
        # Callback groups for threading
        self.service_cb_group = ReentrantCallbackGroup()

        # Camera driver clients (cached)
        self.cam_drivers: Dict[str, CameraDriver] = {}
        
        # Scout light publishers
        self.light_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        
        # Recording state
        self.recording_active = {"left": False, "right": False}
        self.recording_lock = Lock()
        
        # GPS tracking
        self.is_gps_logging = False
        self.gps_log: List[Dict] = []
        self.gps_lock = Lock()
        self.gps_output_path = None


        # Initialize hardware interfaces
        self._setup_camera_drivers()
        self._setup_scout_lights()
        self._setup_gps_logging()


        # Single public service with reentrant callback group
        self.srv = self.create_service(
            CameraRecordSrv, 
            "/antobot_devices_camera/camera_record", 
            self._srv_cb,
            callback_group=self.service_cb_group
        )


    def _load_cfg(self):
        path = self.get_parameter('config_path').get_parameter_value().string_value
        self.get_logger().info(f"Loading config from: {path}")
        with open(path, 'r') as f:
            return yaml.safe_load(f) or {}

    def _setup_camera_drivers(self):
        """Create service clients for individual camera drivers."""
        cameras = self.cfg.get('camera', {})
        
        for loc, _ in cameras.items():
            if loc not in ['left', 'right']:
                self.get_logger().warn(f'Ignoring unknown camera side: {loc}')
                continue
            port = cameras[loc].get('port_core', None)
            self.cam_drivers[loc] = CameraDriver(port=port)

    def _setup_scout_lights(self):
        """Create publishers for scout light control."""
        cameras = self.cfg.get('camera', {})
        
        for loc, config in cameras.items():
            light_topic = config.get('light_topic')
            if light_topic:
                self.light_pubs[loc] = self.create_publisher(Bool, light_topic, 1)
                self.get_logger().info(f'{loc.capitalize()} scout light: {light_topic}')

    def _setup_gps_logging(self):
        """Subscribe to GPS topic if configured."""
        gps_topic = self.cfg.get('gps_topic')
        if gps_topic:
            self.create_subscription(
                NavSatFix,
                gps_topic,
                self._gps_cb,
                qos_profile=qos_profile_sensor_data
            )
            self.get_logger().info(f'  GPS logging: {gps_topic}')

    # ---- Light control ----
    def _set_light(self, loc: str, on: bool):
        pub = self.light_pubs.get(loc)
        if not pub:
            return
        msg = Bool()
        msg.data = bool(on)
        pub.publish(msg)


    # ---- Public service callback ----
    def _srv_cb(self, req, resp):
        cmd = int(req.command)
        cam_num = int(req.camera_num)
        targets = []

        if cam_num == 0:  # BOTH
            targets = ["left", "right"]
        else:
            loc = NUM_TO_LOC.get(cam_num)
            if not loc:
                resp.response_code = False
                resp.response_string = f"Unsupported camera_num={cam_num}. Use 0 (both), 3 (left), 4 (right)."
                return resp
            targets = [loc]

        rec_path = self.cfg.get('recording_path', '/root/ros2_ws/data/')
        
        # Generate absolute basename for recordings
        now = datetime.datetime.now() # local time
        date_folder = now.strftime("%Y%m%d")
        os.makedirs(os.path.join(rec_path, date_folder), exist_ok=True)
        time_str = now.strftime("%H%M%S")
        abs_basename = os.path.join(rec_path, date_folder, f"{time_str}")

        self.get_logger().info(f"CameraRecordManager: cmd={cmd}, cam_num={cam_num}, targets={targets}")
        
        results = []
        # Start recording on each target
        if cmd == 2 and abs_basename:
            self._start_gps_logging(abs_basename)
        
            for loc in targets:
                if self.recording_active[loc]:
                    ok = True
                    msg = "Already recording."
                    results.append((loc, ok, msg))
                    continue
                try:
                    cam = self.cam_drivers.get(loc)
                    self.get_logger().info(f"Start camera {loc}")
                    self.recorders[loc] = Recorder(
                        camera_driver=cam,
                        out_basename=abs_basename+f"_{loc[0]}")
                    self.recorders[loc].start()
                    ok = True
                    msg = f"Recording started"
                    self.get_logger().info(f"{loc.capitalize()} camera recording started.")
                except Exception as e:
                    self.recorders[loc] = None
                    ok = False
                    msg = f"Failed to start: {e}"

                results.append((loc, ok, msg))

                # Light policy per-side
                if ok:  # start
                    with self.recording_lock:
                        self.recording_active[loc] = True
                    self._set_light(loc, True)
        
        # Stop recording on each target
        elif cmd == 3:
            for loc in targets:
                if self.recording_active[loc] == False:
                    ok = True
                    msg = "Not recording."
                    results.append((loc, ok, msg))
                    continue
                try:
                    self.get_logger().info(f"Stop camera {loc}")
                    written = self.recorders[loc].stop()
                    ok = True
                    msg = f"Stopped. Frames={written}"
                    self.get_logger().info(f"{loc.capitalize()} camera recording stopped.")
                finally:
                    self.recorders[loc] = None
                if ok:  # stop
                    with self.recording_lock:
                        self.recording_active[loc] = False
                    self._set_light(loc, False)
                results.append((loc, ok, msg))

        # Stop GPS only if both cameras have stopped
            with self.recording_lock:
                still_recording = any(self.recording_active.values())
            if not still_recording:
                self._stop_gps_logging()

        # Combine results
        if all(ok for _, ok, _ in results):
            resp.response_code = True
            if cmd == 2 and cam_num == 0:
                resp.response_string = "Both cameras: recording started."
            elif cmd == 3 and cam_num == 0:
                resp.response_string = "Both cameras: recording stopped."
            else:
                resp.response_string = "; ".join(f"{loc}: ok" for loc, _, _ in results)
        else:
            resp.response_code = False
            resp.response_string = "; ".join(f"{loc}: {msg}" for loc, ok, msg in results if not ok)

        return resp
    
    # ---- GPS handling ----
    def _gps_cb(self, msg: NavSatFix):
        """Append raw GPS data if logging is active."""
        if not self.is_gps_logging:
            return
        
        with self.gps_lock:
            self.gps_log.append({
                "time_ns": msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec,
                "lat": msg.latitude,
                "lon": msg.longitude,
                "alt": msg.altitude,
            })

    def _start_gps_logging(self, abs_basename: str):
        """Begin collecting GPS messages."""
        if self.is_gps_logging:
            return
        
        self.is_gps_logging = True
        with self.gps_lock:
            self.gps_log.clear()
        
        self.gps_output_path = f"{abs_basename}_gps.json"
        self.get_logger().info(f"Started GPS logging → {self.gps_output_path}")

    def _stop_gps_logging(self):
        """Write all collected GPS samples to a single JSON file."""
        if not self.is_gps_logging:
            return
        
        self.is_gps_logging = False
        
        # Copy data under lock
        with self.gps_lock:
            data = {"gps": list(self.gps_log)}
            num_samples = len(self.gps_log)
            self.gps_log.clear()
        
        # Write outside lock
        json_path = self.gps_output_path
        self.gps_output_path = None
        
        try:
            os.makedirs(os.path.dirname(json_path), exist_ok=True)
            with open(json_path, "w") as f:
                json.dump(data, f)
            self.get_logger().info(f"Wrote GPS log ({num_samples} samples) → {json_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write GPS file: {e}")


def main():
    rclpy.init()
    node = CameraRecordManager()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()