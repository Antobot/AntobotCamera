import os
import datetime
from typing import Dict
from threading import Lock
from antobot_devices_camera.recorder_mkv import Recorder


class CameraRecordManager():
    def __init__(self, config, logger):
        self.logger = logger
        self.cfg = config
        self.recorders: Dict[str, Recorder] = {}
        self.lock = Lock()
        self.recording_active = {"left": False, "right": False}
    
    def _generate_path(self, loc):
        rec_path = self.cfg.get('recording_path', '/home/scouting/acData/camera/data/')
        now = datetime.datetime.now()
        date_folder = now.strftime("%Y%m%d")
        full_dir = os.path.join(rec_path, date_folder)
        os.makedirs(full_dir, exist_ok=True)
        time_str = now.strftime("%H%M%S")
        return os.path.join(full_dir, f"{time_str}_{loc}")

    def start_recording(self, loc, camera_driver):
        """
        Creates and starts a recorder for a specific location.
        Returns: (bool success, string message)
        """
        self.logger.info(f"CameraRecordManager: Attempting to start {loc}")
        
        with self.lock:
            if self.recording_active.get(loc, False):
                return False, "Already recording"

            try:
                # 1. Get Params
                if loc not in self.cfg.get('camera', {}):
                    return False, f"Config missing for camera: {loc}"

                params = self.cfg['camera'][loc]['params']
                fps = params.get('framerate', 30)
                width = params.get('width', 1280)
                height = params.get('height', 720)

                # 2. Prepare Path
                basename = self._generate_path(loc)

                # 3. Init Recorder
                rec = Recorder(basename, width, height, fps)
                
                # 4. Open Writer using Metadata from Driver
                if not camera_driver:
                    return False, "Camera Driver is None"

                rec.open_writer(
                    depth_scale=camera_driver.depth_scale,
                    color_intrinsics=camera_driver.color_intrinsics
                )
                
                # 5. Start Writer Thread
                rec.start()
                
                self.recorders[loc] = rec
                self.recording_active[loc] = True
                self.logger.info(f"CameraRecordManager: Started recording for {loc} at {basename}")
                
                return True, basename

            except Exception as e:
                self.logger.error(f"Start failed for {loc}: {e}")
                return False, str(e)
        
        return False, "Unknown Fallthrough Error"

    def stop_recording(self, loc):
        with self.lock:
            if not self.recording_active.get(loc, False):
                return True, "Not recording"

            rec = self.recorders.get(loc)
            msg = "Stopped"
            
            if rec:
                frames = rec.stop()
                msg = f"Stopped. Frames: {frames}"
                del self.recorders[loc]
            
            self.recording_active[loc] = False
            return True, msg

    def handle_frame(self, loc, color, depth):
        """
        Called by Supervisor's central loop to push data.
        """
        if self.recording_active.get(loc, False):
            try:
                self.recorders[loc].push_frame(color, depth)
            except (KeyError, AttributeError):
                pass