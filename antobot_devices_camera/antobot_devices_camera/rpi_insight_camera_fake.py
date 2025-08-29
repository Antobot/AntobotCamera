import time

class MockPicamera2:
    def __init__(self, tuning=None):
        self.tuning = tuning
        self.started = False
    
    def start(self):
        self.started = True
    
    def close(self):
        self.started = False
    
    def configure(self, config):
        pass
    
    def camera_configuration(self):
        return {
            "controls": {"FrameRate": 30},
            "main": {"size": (2028, 1080), "format": "RGB888"},
            "raw": {"size": (2028, 1080), "format": "SGBRG12"}
        }

    def create_video_configuration(self, *args, **kwargs):
        return {}
    
    def set_controls(self, controls):
        pass
    
    def capture_request(self, flush=False):
        return MockRequest()
    
    @staticmethod
    def load_tuning_file(file_name):
        return {}
    
    @staticmethod
    def find_tuning_algo(tuning_dict, algo_name):
        # Insert mock structure
        tuning_dict.setdefault("rpi.agc", {"channels": [{"exposure_modes": {}}]})
        return tuning_dict["rpi.agc"]["channels"][0]

class MockRequest:
    def release(self):
        pass
    def get_metadata(self):
        return {"SensorTimestamp": 123456789}

# Mock other classes
class MockPreview:
    QTGL = "QTGL"

class MockEncoder:
    def __init__(self):
        self.running = False
        self.output = None
    
    def start(self):
        self.running = True
    
    def stop(self):
        self.running = False
    
    def encode(self, stream_name, request):
        pass

class MockFileOutput:
    def __init__(self, file_path, pts_path=None):
        self.file_path = file_path
        self.pts_path = pts_path

class MockH264Encoder(MockEncoder):
    def __init__(self, bitrate):
        super().__init__()
        self.bitrate = bitrate

# Mock controls
class MockControls:
    class AeConstraintModeEnum:
        Highlight = "Highlight"
    class AeExposureModeEnum:
        Custom = "Custom"
    class AeFlickerModeEnum:
        Manual = "Manual"

# Then redefine your RPiInsightCamera class to use the mocks:
class RPiInsightCamera:
    def __init__(self, preview=False, raw=False, framerate=30):
        # Use the mock classes here
        self.enable_preview = preview
        self.enable_raw = raw

        tuning_file = self.load_tuning_file()
        self.cam = MockPicamera2(tuning=tuning_file)
        
        # Create raw and preview configurations if they have been requested
        if preview:
            lores_config = {'size': (1014,540)}
            display_config = "lores"
        else:
            lores_config = None
            display_config = None

        if raw:
            raw_config = {
                'size': (2028,1080),
                'format': 'SGBRG12'
            }
        else:
            raw_config = None
    
        # Create and apply the camera configuration
        config = self.cam.create_video_configuration(
            sensor={
                'output_size': (2028,1080),
                'bit_depth': 12
            },
            controls={
                'FrameRate': framerate
            },
            main={
                'size': (2028,1080),
                'format': 'RGB888'
            },
            raw=raw_config,
            lores=lores_config,
            display=display_config,
            encode="main",
            buffer_count=12
        )
        self.cam.configure(config)
        
        # Load and set camera control settings
        cam_controls = self.load_camera_controls()
        self.cam.set_controls(cam_controls)

        # Get camera config to set up encoders
        config = self.cam.camera_configuration()

        # Set up encoder for main stream
        self.main_encoder = MockH264Encoder(15000000)
        self.main_encoder.framerate = config["controls"]["FrameRate"]
        self.main_encoder.size = config["main"]["size"]
        self.main_encoder.format = config["main"]["format"]
        
        # Set up encoder for raw stream
        if self.enable_raw:
            self.raw_encoder = MockEncoder()
            self.raw_encoder.framerate = config["controls"]["FrameRate"]
            self.raw_encoder.size = config["raw"]["size"]
            self.raw_encoder.format = config["raw"]["format"]

    def run_frame_capture(self):
        """
        Method to be called from a high-frequecy loop during camera recording.
        Obtains capture request from the camera system and encodes the frame.
        start_recording() must be called first.
        """
        
        # Capture request from camera system
        request = self.cam.capture_request(flush=False)
        
        #TODO: save metadata
        # md = request.get_metadata()
        # metadata.append(md)
        # ts.append(md['SensorTimestamp'])
                
        # Encode frame from the request
        self.main_encoder.encode("main", request)
        if self.enable_raw:
            self.raw_encoder.encode("raw", request)
        
        # Return request to the camera system
        request.release()

        return True

    def is_recording_started(self):
        """
        Returns True if the main recording encoder has started and is ready to
        receive frames to encode and save to a file.
        
        NOTE: The method run_frame_capture must be called repeatedly to get 
        the frames and pass them to the encoders (i.e. to actually record data).
        
        Returns:
            out (bool): True if the main encoder is running
        """
        return self.main_encoder.running
    
    def is_open(self):
        """
        Returns True if the camera is open. 

        If preview is enabled, it will be shown whilst the camera is open.
        
        Returns:
            out (bool): True if the camera is running
        """
        return self.cam.started
    
    def open_camera(self):
        """
        Starts camera and starts preview in a window if requested when
        camera was initialised.
        """
        
        if self.enable_preview:
            self.cam.start_preview(Preview.QTGL)

        self.cam.start()
        
        # Sleep for 1 second to allow camera algorithms to settle before any recording can start
        time.sleep(1) 

    def start_recording(self, filepath):
        """
        Start recording to supplied file path.
        
        Args:
            filepath (str): video save path including file name without extension
        """
        # TODO: error checking on path

        # Append extension to file path and assign encoder output
        full_path_main = f"{filepath}.h264"
        self.main_encoder.output = MockFileOutput(full_path_main)
        
        # Assign raw stream outputs
        if self.enable_raw:
            raw_vid_path = f"{filepath}.raw"
            raw_pts_path = f"{filepath}_pts.txt"
            self.raw_encoder.output = MockFileOutput(raw_vid_path, raw_pts_path)

        # Start encoders
        self.main_encoder.start()
        if self.enable_raw:
            self.raw_encoder.start()

    def stop_recording(self):
        """
        Stop video recording. The camera loop run_frame_capture should have
        finished before calling this method. 
        """
        # Stop encoders
        self.main_encoder.stop()
        if self.enable_raw:
            self.raw_encoder.stop()

        if self.enable_raw:
            pass
            #TODO: save metadata - return to anto_rec?
            # with open(f"{prefix}metadata.txt", 'w') as f:
            # json.dump(metadata, f)

    def close_camera(self):
        """
        Close the camera, including preview if enabled.
        """
        self.cam.close()

    def load_tuning_file(self):
        """
        Load tuning file for IMX477 sensor and modify with a custom exposure mode.

        Returns:
            tuning: tuning file for IMX477 used for robot strawberry insight
        """
        tuning = MockPicamera2.load_tuning_file("imx477.json")
        algo = MockPicamera2.find_tuning_algo(tuning, "rpi.agc")
        return tuning
    
    def load_camera_controls(self):
        """
        Create dictionary with camera control settings.
        
        Returns:
            cam_controls (dict): dictionary to be passed to picam2.set_controls
        """
        cam_controls = {
            "AwbEnable": True,
            "AeEnable": True,
            "AeConstraintMode": MockControls.AeConstraintModeEnum.Highlight,
            "AeExposureMode": MockControls.AeExposureModeEnum.Custom,
            "AeFlickerMode": MockControls.AeFlickerModeEnum.Manual,
            "AeFlickerPeriod": 10000
        }
        return cam_controls