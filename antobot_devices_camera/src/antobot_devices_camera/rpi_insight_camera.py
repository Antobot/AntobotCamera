#!/usr/bin python3
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
import threading
import numpy as np
import math

from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, Encoder
from picamera2.outputs import FileOutput
from libcamera import controls

from aiortc import VideoStreamTrack
from av import VideoFrame


class CameraStreamTrack(VideoStreamTrack):
    """
    A video track that captures frames from a callback to the latest pi camera request
    """
    def __init__(self, read_array_callback, frame_dims, rotation=0):
        """
        Initialise a CameraStreamTrack

        Args:
            read_array_callback (callback): callback to read frame from the camera
            frame_dims (tuple): dimensions of camera frame (height, width)   
        """
        super().__init__()
        
        # Callback to read frame from the camera
        self.read_frame = read_array_callback
        
        # Dimensions that the camera records at
        # (height, width) (assuming portrait after a 90 deg rotation)
        self.camera_dims = frame_dims
        
        # Placeholder for stream dimensions, calculated and updated when stream is requested
        # (height, width)
        self.stream_dims = None
        
        # Attribute to be read by external streamer
        self.rotation = rotation

    def set_size(self, width, height):
        """
        Calculates and saves the appropriate stream size given the dimensions of the container on the webpage.

        Args:
            width (int): maximum width in px permitted for the stream 
            height (int): maximum height in px permitted for the stream       
        """
        # container dims on webpage
        hc = height
        wc = width

        # camera frame dims
        hf = self.camera_dims[0]
        wf = self.camera_dims[1]

        # set stream size to limiting height/width
        if hf/hc > wf/wc:
            height = hc
            width = math.floor(wf/hf * height)
        else:
            width = wc
            height = math.floor(hf/wf * width)
        
        width = width - width%2
        height = height - height%2

        self.stream_dims = (height, width)

    async def recv(self):
        """
        Returns frame for webRTC stream when called.

        Reads frame from camera using provided callback, rotates and resizes. 
        Returns green frame if the callback doesn't yield a frame.

        Returns:
            video_frame (VideoFrame): encoded frame   
        """
        pts, time_base = await self.next_timestamp()

        # Read latest frame from RPiInsightCam
        frame = self.read_frame(stream='lores')  

        # If frame is none, return green
        if frame is not None:
            # to maintain backward compatibility, rotate if the stream dimensions have been set
            # use rotation attribute to determine the number of 90 degree rotations
            if self.stream_dims is not None:
                frame = np.rot90(frame, -self.rotation // 90)
            
            # convert to video frame
            video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
            
            # to maintain backward compatibility, resize if the stream dimensions have been set
            if self.stream_dims is not None:
                video_frame = video_frame.reformat(self.stream_dims[1], self.stream_dims[0])
            
        else:
            video_frame = VideoFrame(width=self.stream_dims[1], height=self.stream_dims[0])

        video_frame.pts = pts
        video_frame.time_base = time_base
        
        return video_frame

    def stop(self):
        # catch stop from receiver and keep alive
        print('CameraStreamTrack stop caught. Keeping alive.')

    def close(self):
        print('Stopping CameraStreamTrack.')
        super().stop()
        

class RPiInsightCamera:
    def __init__(self, preview=False, raw=False, framerate=30, frame_dims=None, cam={'num': 0, 'model': 'imx296'}, rotation=0):
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
        self.requested_frame_dims = frame_dims
        self.rotation = rotation
        
        self.cam_num = cam['num']
        self.cam_model = cam['model']

        self.frame_lock = threading.Lock() # lock whilst a frame is being processed/encoded 
        self.request_lock = threading.Lock() # lock for reading/writing picamera requests

        self.latest_request = None 
        self.stream_track = None

        self.close_flag = True # signal that cam loop should close
        self.record_flag = False # signal that cam loop should record (encode) frames
        self.camera_loop_thread = None
        self.frame_metadata = [] # each entry is a dictionary of metadata for each frame

        # let's init to check we can open it and setup encoders etc.
        self.init_camera()
        self.cam.close()
    
    def init_camera(self):

        # NB, once a camera closed, need to make a new instance of Picamera2() to open again

        if self.cam_model == 'imx296':
            self.frame_dims = (1456, 1088)
            self.raw_format = 'SGBRG10'
            self.bit_depth = 10
            self.preview_size = (728, 544)
        elif self.cam_model == 'imx477':
            self.frame_dims = self.requested_frame_dims if self.requested_frame_dims is not None else (2028, 1080)
            self.raw_format = 'SBGGR12'
            self.bit_depth = 12
            # set preview size to 640px long edge, keeping aspect ratio
            aspect_ratio = self.frame_dims[0] / self.frame_dims[1]
            height = 640 # long edge
            self.preview_size = (height, int(height / aspect_ratio)) #(1014, 540)


        # Create camera object with custom tuning file
        self.tuning_file = self.cam_model + '.json'
        tuning_file = self.load_tuning_file()
        self.cam = Picamera2(camera_num=self.cam_num, tuning=tuning_file)

        # Create raw and preview configurations if they have been requested
        if self.enable_preview:
            lores_config = {'size': self.preview_size, 'format': 'RGB888'}
            display_config = None
        else:
            lores_config = None
            display_config = None

        if self.enable_raw:
            raw_config = {
                'size': self.frame_dims,
                'format': self.raw_format
            }
        else:
            raw_config = None
    
        # Create and apply the camera configuration
        config = self.cam.create_video_configuration(
            # Put camera sensor into mode 1 (i.e cam.sensor_modes[1]).
            # The best way is to specify output_size and bit_depth 
            # (Picamera2 docs, p.23)
            sensor={
                'output_size': self.frame_dims,
                'bit_depth': self.bit_depth
            }, 
            # set frame rate; 50 fps max in this sensor mode
            controls={
                'FrameRate': self.framerate
            },
            main={
                'size': self.frame_dims,
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

        # Create video track for stream
        self.stream_track = CameraStreamTrack(self.read_request_array, self.frame_dims, self.rotation)

    def read_request_array(self, stream='main'):
        """
        Callback to read the latest frame as an array.

        Returns:
            out ( array | None ) : numpy array of image or None if no request available
            stream (str): 'main', 'raw' or 'lores'
        """
        if self.latest_request is not None:
            with self.request_lock:
                return self.latest_request.make_array(stream)
        else:
            return None

    def capture_still(self, filepath):
        """
        Save still image from main as jpg and raw as dng if enabled.
            
        Args:
            filepath (str): file path to save still image

        Returns:
            metadata (dict): metadata of the still image
        """
        if self.latest_request is not None:
            with self.request_lock:

                self.latest_request.save("main", f"{filepath}_{self.cam_num}.jpg")
                if self.enable_raw:
                    self.latest_request.save_dng(f"{filepath}_{self.cam_num}.dng")

                return self.latest_request.get_metadata()
        else:
            return None
        
    
    def camera_loop(self):
        """
        Camera loop drives all camera functions and is to be run in a thread. 
        
        One iteration handles one frame. When running, it save requests for stream.
        When the record flag is set, it encodes requests and stores metadata.
        """

        # expected_interval = 1.0 / self.framerate
        # print(f"[CameraLoop] Target frame interval: {expected_interval:.3f} sec")

        # frame_count = 0
        # total_encode_time = 0
        # last_fps_time = time.time()

        # Run loop until close flag is set
        while not self.close_flag:
            # loop_start = time.time()
            
            # Capture request from camera system
            request = self.cam.capture_request(flush=False)
            
            # Update saved request
            with self.request_lock:
                self.latest_request.release()
                self.latest_request = request

            # Use a lock to ensure a frame is fully processed (all encoders and metadata)
            # Note, whilst each pi camera function (e.g. encode) is thread safe, we don't want a mismatch of metadata and video frames
            with self.frame_lock:
                if self.record_flag:
                    # encode_start = time.time()

                    # Get camera metadata
                    md = request.get_metadata()
                    md_keys = ("SensorTimestamp", "ExposureTime")
                    # md_keys = ("SensorTimestamp", "ExposureTime", "AnalogueGain", "Lux", "ColourGains")
                            
                    # Encode frame from the request
                    self.main_encoder.encode("main", request)
                    
                    # encode_end = time.time()
                    # encode_time = encode_end - encode_start
                    # total_encode_time += encode_time
                    # print(f"[ENCODE] Frame {frame_count} took {encode_time*1000:.2f} ms")
                    
                    # If raw is enabled, encode frame and use all metadata keys
                    if self.enable_raw:
                        self.raw_encoder.encode("raw", request)
                        md_keys = md.keys()
                    
                    # Add this frame's metadata
                    self.frame_metadata.append({k: md[k] for k in md_keys})


            # loop_end = time.time()
            # loop_time = loop_end - loop_start

            # if loop_time > expected_interval:
            #     print(f"[⚠] Slow loop: {loop_time:.3f}s (expected {expected_interval:.3f}s)")

            # frame_count += 1
            # if frame_count % 50 == 0:
            #     elapsed = loop_end - last_fps_time
            #     print(f"[FPS] ~{50 / elapsed:.2f} fps | Avg encode: {total_encode_time/50*1000:.2f} ms")
            #     total_encode_time = 0
            #     last_fps_time = time.time()

    def is_recording(self):
        """
        Returns True if the camera is open, the main recording encoder has started and the record flag is set.
        
        Returns:
            out (bool): True if recording else False
        """
        return self.is_open() and self.main_encoder.running and self.record_flag
    

    def is_open(self):
        """
        Returns True if the camera is open. 
     
        Returns:
            out (bool): True if the camera is running
        """
        return self.cam.started and self.cam_loop_thread.is_alive()
    

    def open_camera(self):
        """
        Starts camera and starts preview in a window if requested when
        camera was initialised.
        """
        
        #Init camera here, rather than with class
        self.init_camera()

        # if self.enable_preview:
        #     self.cam.start_preview(Preview.QTGL)

        self.cam.start()
        
        # Sleep for 1 second to allow camera algorithms to settle before any recording can start
        time.sleep(1) 

        # Flush camera requests and set initial request
        self.latest_request = self.cam.capture_request(flush=True)

        # Create and start camera loop thread
        self.close_flag = False
        self.record_flag = False
        self.cam_loop_thread = threading.Thread(target=self.camera_loop)
        self.cam_loop_thread.start()


    def start_recording(self, filepath):
        """
        Start recording to supplied file path.
        
        Args:
            filepath (str): video save path including file name without extension
        """
        # TODO: error checking on path

        # Use frame lock so we don't reconfigure encoders whilst they are being written
        with self.frame_lock:
            # Append extension to file path and assign encoder output
            full_path_main = f"{filepath}_{self.cam_num}.h264"
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

            # Clear metadata
            self.frame_metadata = []

            # Set flag so thread encodes frames
            self.record_flag = True


    def stop_recording(self):
        """
        Stop video recording and return camera frame metadata.

        Returns:
            metadata (list): list where each item is a dictionary of metadata for one frame
        """
        
        # Use frame lock so we don't stop encoders until frame is fully processed
        with self.frame_lock:
            # Flag camera thread to stop recording
            self.record_flag = False

            # Stop encoders
            self.main_encoder.stop()
            if self.enable_raw:
                self.raw_encoder.stop()

        # Return metadata to camera_record
        return self.frame_metadata

    def close_camera(self):
        """
        Close the camera.
        """

        # End the stream track
        self.stream_track.close()

        # Stop camera thread
        self.close_flag = True
        self.cam_loop_thread.join()

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
        tuning = Picamera2.load_tuning_file(self.tuning_file)
        algo = Picamera2.find_tuning_algo(tuning, "rpi.agc")
        algo["channels"][0]["exposure_modes"]["normal"] = { #NOTE: workaround for libcamera bug to update normal exposure mode
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
        # Due to the current bug with libcamera, we are using and have modified the normal exposure mode
        cam_controls = {
            "AwbEnable": True,
            "AeEnable": True,
            "AeConstraintMode": controls.AeConstraintModeEnum.Highlight,
            "AeExposureMode": controls.AeExposureModeEnum.Normal,
            "AeFlickerMode": controls.AeFlickerModeEnum.Manual,
            "AeFlickerPeriod": 10000
        }
        return cam_controls
