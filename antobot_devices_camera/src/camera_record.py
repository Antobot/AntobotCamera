#! /usr/bin python3
# Copyright (c) 2022, ANTOBOT LTD.
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

# # # Code Description:     This code contains the main program for svo file and corresponding json file recording.
# # # Interfaces:           _serviceCallbackcameraRecord - receive request from anto_manager.av_cam_mgr and send response.

# Contacts: Authors:    jinhuan.liu@antobot.ai
#                       james.bennett@antobot.ai
#           Owner:      jinhuan.liu@antobot.ai


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import os
import time
import yaml
import json
import psutil
import shutil
import threading
from pathlib import Path
from signal import signal, SIGINT
from datetime import datetime, timedelta
from datetime import time as t

import rospy
import rospkg
import rostopic
import tf2_ros
from antobot_camera_msgs.srv import cameraRecord, cameraRecordResponse
from sensor_msgs.msg import NavSatFix   

from antobot_devices_camera.rpi_insight_camera import RPiInsightCamera


def is_master_running():
    """
    Function to check the connection with ROS Master. Runs every 10s.
    """

    while True:
        time.sleep(10)
        try:
            rostopic.get_topic_class('/rosout')
        except rostopic.ROSTopicIOException as e:
            rospy.loginfo(f"SW4102 cameraRecord: Lost connection with ROS Master")
            print(
                "Kill the current process, if you use your own laptop, please rerun the code. (For carrier board, service will be restarted automatically.)")
            os._exit(1)


class camRecord:
    def __init__(self, argv=None):
        """
        Initialises a new cameraRecord class object.

        Args:
            argv (sys.argv): system arguments

        """

        self.save_path = os.path.join(os.path.dirname(os.getcwd()), 'saved_recordings')

        # Read config
        rospack = rospkg.RosPack()
        try:
            path = rospack.get_path('antobot_description')
            with open(path + '/config/platform_config.yaml', 'r') as file:
                params = yaml.safe_load(file)

        except Exception as e:
            print(f"Failed to read robot config file, error: {e}")
            raise

        # Create and setup camera
        try:
            if "camera" in params:
                for cam_type in params["camera"]:
                    mode = params["camera"][cam_type]["mode"]
                    cam_position = params["camera"][cam_type]["location"]

                    self.cam_name = f'RP_{cam_position}'
                    self.srv_name = f"/antobot_devices_camera/{cam_type}/{mode}/{cam_position}"
                    
                    if "dual" in params["camera"][cam_type].keys() and params["camera"][cam_type]["dual"] is True:
                        # make 2 cameras
                        self.cams = [
                            RPiInsightCamera(preview=False, raw=False, framerate=30, cam_num=0),
                            RPiInsightCamera(preview=False, raw=False, framerate=30, cam_num=1)
                        ]

                    else:
                        #make one camera
                        self.cams = [
                            RPiInsightCamera(preview=False, raw=False, framerate=30, cam_num=0)
                            , 
                        ]

                    # only supporting one camera
                    break
        except KeyError as e:
            print(f"KeyError in camera setup. Is platform_config.yaml properly defined? Error: {e}")
            raise
        except Exception as e:
            print(f"Failed to setup camera(s). Error: {e}")
            raise

        # Setup GPS logging 
        try:
            if "gps" in params:
                self.use_gps = True
            
                if "urcu" in params["gps"]:
                    self.robot_gps_sub = rospy.Subscriber("/am_gps_urcu", NavSatFix, self.gps_callback)
                elif "scouting_box" in params["gps"]:
                    self.robot_gps_sub = rospy.Subscriber("/antobot_f9p_usb", NavSatFix, self.gps_callback)
                else:
                    # There is a gps key but no key for the platform type. 
                    raise ValueError("platform_config.yaml has a GPS key but there is no key for the platform type.")
        except KeyError as e:
            print(f"KeyError in GPS logging setup. Is platform_config.yaml properly defined? Error: {e}")
            raise
        except Exception as e:
            print(f"Failed to setup GPS logging. Error: {e}")
            raise
        finally:
            self.gps = []

        # Create and set up stream
        self.enable_stream = True
        if self.enable_stream:
            from preview_streamer import PreviewStreamer
            self.streamer = PreviewStreamer(self.cams[0].stream_track)
        else:
            self.streamer = None
       
        self.output_basename = None

        self.display_image = False

        self.tfBuffer = None
        self.listener = None

        rospy.init_node(self.cam_name, anonymous=False)
        self.srvcameraRecord = rospy.Service(self.srv_name, cameraRecord, self._serviceCallbackcameraRecord)
        self.json_dict = self.init_metadata()
        
        self.master_check_thread = threading.Thread(target=is_master_running)
        self.master_check_thread.start()

        signal(SIGINT, self.signal_handler)  # Allow interrupt from keyboard (CTRL + C).

        # rospy.spin()

    
    def manage_disk_space(self):
        # check disk usage
        disk_free_space = psutil.disk_usage('/').free / (1024 ** 3)
        print(f"free space: {disk_free_space}G")
        while disk_free_space < 24:  # if disk usage is < 1GB, clean backup directory
            self.free_space()
            disk_free_space = psutil.disk_usage('/').free / (1024 ** 3)

        # TODO: this function doesn't work if backup is already empty....


    def init_metadata(self):
        """
        Initialises a dictionary to save map origin and map to zed transforms

        Returns:
            json_dict (dict): initialised dictionary including map origin

        """
        # Are these lines used?
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Time
        time_sync = {
            'rospy_now': rospy.Time.now().to_nsec(),
            'monotonic_now': time.monotonic_ns()
        }
        # Initialise a dictionary to convert ros msg to json file
        json_dict = {
            'origin': {'longitude': 0, 'latitude': 0},
            'gps': [],
            'cam_metadata': [],
            'time_sync': time_sync,
            'cam_name': self.cam_name
        }

        return json_dict
    

    def _serviceCallbackcameraRecord(self, request):
        """
        Callback function to handle the service request from cam manager.

        Args:
            request (cameraRecordRequest): command (int8)   0 - open cam, 1 - close cam,
                                                       2 - start recording, 3 - stop recording
        Returns:
            return_msg (cameraRecordResponse): responseCode (bool)
                                          responseString (string)

        """
        return_msg = cameraRecordResponse()

        # -------------------
        #     OPEN CAMERA
        # -------------------
        if request.command == 0:  
            success = self.open_camera()

            if success:
                return_msg.responseCode = True
                return_msg.responseString = f"{self.cam_name} is open."
            else:
                return_msg.responseCode = False
                return_msg.responseString = f"Cannot open {self.cam_name}."
        
        # --------------------
        #     CLOSE CAMERA
        # --------------------
        elif request.command == 1:
            success = self.close_camera()

            if success:
                return_msg.responseCode = True
                return_msg.responseString = f"{self.cam_name} has been closed."
            else:
                return_msg.responseCode = False
                return_msg.responseString = f"Cannot close {self.cam_name}."

        # -----------------------
        #     START RECORDING
        # -----------------------
        elif request.command == 2:

            # update recording directory if the raspberry pi is not master device
            rec_path = request.recordingBasename
            name_start = rec_path.find('AntoManager')
            pkg_path = rospkg.RosPack().get_path('antobot_devices_camera')
            # Go up two directories
            package_root = os.path.abspath(os.path.join(pkg_path, '..', '..'))

            self.output_basename = os.path.join(package_root, rec_path[name_start:])
            rospy.loginfo(self.output_basename)

            success = self.start_recording()

            if success:
                return_msg.responseCode = True
                return_msg.responseString = f"{self.cam_name} recording started, use Ctrl-C or send command to stop."
            else:
                return_msg.responseCode = False
                return_msg.responseString = f"{self.cam_name} failed to start recording."
        
        # ----------------------
        #     STOP RECORDING
        # ----------------------
        elif request.command == 3:
            success = self.stop_recording()

            if success:
                return_msg.responseCode = True
                return_msg.responseString = f"{self.cam_name} recording stopped."
            else:
                return_msg.responseCode = False
                return_msg.responseString = f"{self.cam_name} failed to stop recording."
        

        rospy.loginfo(f'SW4102: cameraRecord: Camera Request Command: {request.command}')
        rospy.loginfo(f'SW4102: cameraRecord: Camera Response : {return_msg.responseString}')

        return return_msg

    def open_camera(self):
        """
        Opens the camera and displays preview if initialised.

        Returns:
            success (bool) : True if camera is open, False if it didn't open

        """

        # If the camera is already opened, return straight away
        for cam in self.cams:
            if not cam.is_open():
                break
        else:
            return True

        # Try to open camera
        for cam in self.cams:
            cam.open_camera()

        # Check the camera opened, if not return False
        for cam in self.cams:
            if not cam.is_open():
                return False
        else:
            return True
            
        

    def close_camera(self):
        """
        Close the camera. If recording hasn't been stopped, it will stop it first.

        Returns:
            success (bool) : True if camera is closed, False if it didn't close
        """

        for cam in self.cams:
            if cam.is_open():
                break
        else:
            return True
        
         # If recording hasn't been stopped, stop it first
        if self.is_cam_recording(): 
            self.stop_recording()

        
        for cam in self.cams:
            if cam.is_open():
                cam.close_camera()

        for cam in self.cams:
            if cam.is_open():
                return False
        else:
            return True

    def start_recording(self):
        """
        Instructs camera to start recording video (and metadata? inc. gps?).

        Returns:
            success (bool): True if recording is started, False if recording failed to start
        """
        
        # check if camera is opened, if not, open camera first
        for cam in self.cams:
            if not cam.is_open():
                if not self.open_camera():
                    return False

        # if the camera is already recording, return straight away
        if self.is_cam_recording():
            return True

        # clear dict
        self.json_dict = self.init_metadata()

        if self.use_gps:
            self.json_dict['origin']['latitude'] = rospy.get_param('/GPS_origin/latitude',0)
            self.json_dict['origin']['longitude'] = rospy.get_param('/GPS_origin/longitude',0)
            self.json_dict['gps'] = []

        # Setup and start encoders
        
        for cam in self.cams:
            cam.start_recording(self.output_basename)

        # Check recording has started
        if self.is_cam_recording():
            return True
        else:
            return False
        

    def stop_recording(self):
        """
        Function to stop recording and save camera position to json file.

        Returns:
            success (bool): True if recording is stopped, False if recording failed to stop
        """
        
        # if the camera is already stopped, return straight away
        if not self.is_cam_recording():
            return True
        
        # if recording hasn't been stopped, stop it
        if self.is_cam_recording():
            
            metadata_list = []

            # Stop recording and store camera per frame metadata
            for cam in self.cams:
                md = cam.stop_recording()
                metadata_list.append(md)
            
            self.json_dict['cam_metadata'] = metadata_list
            self.write_metadata()

        # Check recording has stopped
        if not self.is_cam_recording():
            return True
        else:
            return False


    def is_cam_recording(self):
        """
        Returns True if the camera is recording, otherwise False.

        Returns:
            out (bool): True if camera is recording
        """
        for cam in self.cams:
            if not cam.is_recording():
                return False
        
        return True


    def signal_handler(self, signal_received, frame):
        """
        Function to handle interrupt from keyboard (CTRL + C). Stop recording and close camera.

        Args:
            signal_received (): the signal number
            frame ():  the current stack frame (None or a frame object)

        """
        self.close_camera()
        exit(0)

    def gps_callback(self, msg):
        
        if self.use_gps:
            # Put data from message into dictionary
            entry = {
                'time': msg.header.stamp.to_nsec(),
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude
            }

            # Write to metadata dict
            self.json_dict['gps'].append(entry)

    def write_metadata(self):
        """
        Dump metadata for one recording to a json file.

        """
        filename = f"{self.output_basename}.json"

        try:
            with open(filename, "w") as f:
                json.dump(self.json_dict, f)  # writes the camera pose as a json file
        except rospy.ROSInterruptException:
            pass

  
    def free_space(self):
        try:
            # Ensure the directory exists
            backup_path = os.path.join(self.save_path, 'backup')
            if not os.path.exists(backup_path):
                print(f"Directory '{backup_path}' does not exist.")
                return

            # List all files in the directory
            files = os.listdir(backup_path)
            files.sort()

            # Delete each file in the directory
            if len(files):
                file_path = os.path.join(backup_path, files[-1])
                shutil.rmtree(file_path)
                rospy.loginfo(f"SW4102: cameraRecord: Deleted file: {file_path}")
        except Exception as e:
            rospy.loginfo(f"SW4102: cameraRecord: Deleted file failed: {str(e)}")


if __name__ == "__main__":
    try:
        rospy.loginfo(f"SW4100: cameraRecord Node launched")
        avRec = camRecord()
   
        if avRec.enable_stream:
            # if streaming, use aiohttp event loop
            avRec.streamer.run()
        else:
            # else, keep alive with rospy.spin
            rospy.spin()
    
    except Exception as e:
        print(e)
        rospy.loginfo(f"SW4101: cameraRecord Node died: {e}")
