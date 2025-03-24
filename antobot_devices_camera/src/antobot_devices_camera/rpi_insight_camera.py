#!/usr/bin/python3
# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description: Configures the Raspberry Pi camera and provides open, close, start, stop functionality tailored to robot scouting needs.
# # # Interfaces:       Imported and called by anto_rec.py

# Contacts: Authors:    james.bennett@antobot.ai
#           Owner:      james.bennett@antobot.ai


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import time

from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, Encoder
from picamera2.outputs import FileOutput
from libcamera import controls


class RPiInsightCamera:
    def __init__(self, preview=False, raw=False, framerate=30):
        """
        Initialise Raspberry Pi camera configured for robot scouting.

        The `main` stream is always configured, `raw` and `preview` streams are
        enabled based on the arguments. The camera is configured with custom
        settings suited to the task of imaging strawberries in a polytunnel
        from a moving robot.

        Args:
            preview (bool): enable preview stream
            raw (bool): enable saving raw stream
            framerate (int): requested camera frames per second, max 50 fps
        """

        # Save requested streams
        self.enable_preview = preview
        self.enable_raw = raw

        # Attributes
        self.vid_extension = 'h264'
        self.framerate = framerate

        # let's init to check we can open it and setup encoders etc.
        self.init_camera()
        self.cam.close()
    
    def init_camera(self):

        # NB, once a camera closed, need to make a new instance of Picamera2() to open again
        # TODO: error check whether a camera is already open

        # Create camera object with custom tuning file
        tuning_file = self.load_tuning_file()
        self.cam = Picamera2(tuning=tuning_file)

        # Create raw and preview configurations if they have been requested
        if self.enable_preview:
            lores_config = {'size': (1014,540)}
            display_config = "lores"
        else:
            lores_config = None
            display_config = None

        if self.enable_raw:
            raw_config = {
                'size': (2028,1080),
                'format': 'SGBRG12'
            }
        else:
            raw_config = None
    
        # Create and apply the camera configuration
        config = self.cam.create_video_configuration(
            # Put camera sensor into mode 1 (i.e cam.sensor_modes[1]).
            # The best way is to specify output_size and bit_depth 
            # (Picamera2 docs, p.23)
            sensor={
                'output_size': (2028,1080),
                'bit_depth': 12
            }, 
            # set frame rate; 50 fps max in this sensor mode
            controls={
                'FrameRate': self.framerate
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
        self.main_encoder = H264Encoder(15000000)
        self.main_encoder.framerate = config["controls"]["FrameRate"]
        self.main_encoder.size = config["main"]["size"]
        self.main_encoder.format = config["main"]["format"]
        
        # Set up encoder for raw stream
        if self.enable_raw:
            
            # This encoder essentially does nothing 
            self.raw_encoder = Encoder()
            self.raw_encoder.framerate = config["controls"]["FrameRate"]
            self.raw_encoder.size = config["raw"]["size"]
            self.raw_encoder.format = config["raw"]["format"]

    def run_frame_capture(self):
        """
        Method to be called from a high-frequecy loop during camera recording.
        Obtains capture request from the camera system and encodes the frame.
        `start_recording()` must be called first.
        """
        
        # Capture request from camera system
        request = self.cam.capture_request(flush=False)
        
        # Get camera metadata
        md = request.get_metadata()
        md_keys = ("SensorTimestamp",)
        # md_keys = ("SensorTimestamp", "ExposureTime", "AnalogueGain", "Lux", "ColourGains")
                
        # Encode frame from the request
        self.main_encoder.encode("main", request)
        
        # If raw is enabled, encode frame and use all metadata keys
        if self.enable_raw:
            self.raw_encoder.encode("raw", request)
            md_keys = md.keys()
        
        # Return request to the camera system
        request.release()

        return {k: md[k] for k in md_keys}

    def is_recording_started(self):
        """
        Returns True if the main recording encoder has started and is ready to
        receive frames to encode and save to a file.
        
        NOTE: The method `run_frame_capture` must be called repeatedly to get 
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
        Starts the camera and initializes preview if requested.
        If an error occurs, it reinitializes the camera.
        """
        
        #Init camera here, rather than with class
        self.init_camera()

        if self.enable_preview:
            self.cam.start_preview(Preview.QTGL)

            # Fully reinitialize the camera
            self.cam = None  # Remove the old instance
            self.init_cam()

            if self.enable_preview:
                self.cam.start_preview(Preview.QTGL)

            self.cam.start()
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
        self.main_encoder.output = FileOutput(full_path_main)
        
        # Assign raw stream outputs
        if self.enable_raw:
            
            raw_vid_path = f"{filepath}.raw"
            raw_pts_path = f"{filepath}_pts.txt"
            self.raw_encoder.output = FileOutput(raw_vid_path, raw_pts_path)

        # Start encoders
        self.main_encoder.start()
        if self.enable_raw:
            self.raw_encoder.start()


    def stop_recording(self):
        """
        Stop video recording. The camera loop `run_frame_capture` should have
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

        # The exposure mode specifies how the desired exposure of the agc/aec
        # algorithm is divided between exposure time and gain. Partially
        # documented on p.34 of Raspberry Pi Camera Algorithm and Tuning Guide
        # 
        # From testing, 
        #  > 0th value of shutter and gain are set
        #  > holds gain at 0th value, ramps to 1st shutter value 
        #  > holds shutter at 1st value, ramps to 1st gain value
        #  > alternate ramping to shutter and gain values, maxing out at final values in list
        tuning = Picamera2.load_tuning_file("imx477.json")
        algo = Picamera2.find_tuning_algo(tuning, "rpi.agc")
        algo["channels"][0]["exposure_modes"]["custom"] = {
            "shutter": [100, 1000, 2000, 5000, 10000], 
            "gain": [1.0, 8.0, 12.0, 16.0, 80.0]
        }
        return tuning
    
    def load_camera_controls(self):
        """
        Create dictionary with camera control settings.
        
        Returns:
            cam_controls (dict): dictionary to be passed to picam2.set_controls
        """

        #NOTE: in future, this method could take arguments to supply different control values for different scenarios, or load from a file

        # Set camera controls, including use custom exposure mode
        # Ignore the warnings about custom exposure mode, it does appear use the right settings
        cam_controls = {
            "AwbEnable": True,
            "AeEnable": True,
            "AeConstraintMode": controls.AeConstraintModeEnum.Highlight,
            "AeExposureMode": controls.AeExposureModeEnum.Custom,
            "AeFlickerMode": controls.AeFlickerModeEnum.Manual,
            "AeFlickerPeriod": 10000
        }
        return cam_controls