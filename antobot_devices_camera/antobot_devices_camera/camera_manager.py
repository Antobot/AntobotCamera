#! /usr/bin/env python3

# Copyright (c) 2022, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     This code manages the different cameras on Antobot's robots. It receives as requests to
# # #                       open/close and/or use cameras, and then starts the appropriate scripts based on the request,
# # #                       or passes along a request to another script, when appropriate.
# # #                       It returns whether the request was successful or not.

# Contacts: daniel.freer@antobot.ai
#           william.eaton@antobot.ai
#           meiru.zhang@antobot.ai
#           jinhuan.liu@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import sys
import yaml
import rclpy
from datetime import datetime
from std_msgs.msg import Bool, String

from antobot_camera_msgs.srv import CamManager
from antobot_camera_msgs.srv import CameraRecord
from cameraRecordClient import cameraRecordClient

from ament_index_python.packages import get_package_share_directory




class cameraManager:
    def __init__(self):
        ##############################################################################
        ## Check for the simulation parameter which should be set by am_sim     
        ##############################################################################

        self.cameras = {}
        self.read_config_file()
        try:
            self.sim = rclpy.get_param("/simulation")

        except:
            self.sim=False # If the simulation parameter has not been assigned, assume not a simulation


        ##############################################################################
        ## Run either real or simulated manager    
        ##############################################################################

        if self.sim:
            print('Camera Manager: This is a simulation - using fake camera calls.')
        else:
            print('Camera Manager: This is not a simulation - using real ZED2/Raspberry pi camera commands.')


        # Create a service to allow other nodes to start/stop cameras
        self.srvCamMgr = rclpy.Service("/antobot/camera_manager/camera", CamManager, self._serviceCallbackCamMgr)

        
        
        self.pub_scout_light = rclpy.Publisher("/antobot_manager_device/scout_light",Bool, queue_size=1)

    def read_config_file(self):
        # rospack = rospkg.RosPack()

        try:
            path = get_package_share_directory('antobot_description')
            # path = rospack.get_path('antobot_description')
            with open(path + '/config/platform_config.yaml', 'r') as file:
                params = yaml.safe_load(file)

            if "camera" in params:
                for cam_type in params["camera"]:
                    mode = params["camera"][cam_type]["mode"]
                    location = params["camera"][cam_type]["location"]

                    serviceName = f"/antobot_devices_camera/{cam_type}/{mode}/{location}"

                    if location not in self.cameras:
                        self.cameras[location] = {}

                    self.cameras[location][cam_type] = Camera(cam_type, mode, location, serviceName)

                    # Todo: launch corresponding camera node on carrier board
            else:
                rclpy.loginfo(f'SW2312: Camera Manager: No camera settings found in config')

        except Exception as e:
            print(f"Failed to read robot config file, error: {e}")

    ############################################################################################
    ## RosService callback - This is the main method of interaction (also works with simulation)
    ############################################################################################

    def _serviceCallbackCamMgr(self, request):
       
        ## ROS service input:
        #int8 camera_num		# 1 - front, 2 - back, 3 - left, 4 - right
        #int8 command			# 0 - stop cameras
                                # 1 - start front or back cameras using ROS launch for antomove
                                # 2 - toggle camera open state
                                # 3 - toggle camera recording state

        
        ## ROS Service response:
        #int8 responseCode		# 1 - success, 0 - failure
        #string responseString	# Additional info

        # Create the return message
        return_msg = CamManager.Response
        cams = None

        recording_basename = request.recording_basename

        if request.command == 2:  # toggle camera open state
            if request.camera_num == 3:  # left camera
                cams = self.cameras['left']

            elif request.camera_num == 4: # right camera
                cams = self.cameras['right']

            for cam in cams.values():
                if cam:
                    rclpy.loginfo(f'SW2312: Camera Manager: {cam.location} {cam.camType} camera open state: {cam.isOpen}')
                    rclpy.loginfo(f'SW2312: Camera Manager: Make request to toggle {cam.location} {cam.camType} camera open state')
                    response = cam.toggleOpen()
                    return_msg.responseCode = response.responseCode
                    return_msg.responseString = response.responseString


        elif request.command == 3:  # toggle camera recording state
            if request.camera_num == 3:  # left camera
                cams = self.cameras['left']

            elif request.camera_num == 4:  # right camera
                cams = self.cameras['right']

            for cam in cams.values():
                rclpy.loginfo(f'SW2312: Camera Manager: {cam.location} {cam.camType} camera recording state: {cam.isRecording}')
                rclpy.loginfo(f'SW2312: Camera Manager: Make request to toggle {cam.location} {cam.camType} camera recording state')
                response = cam.toggleRecording(recording_basename)
                return_msg.responseCode = response.responseCode
                return_msg.responseString = response.responseString

                if cam.camType == 'zed':  # only check the zed status for now
                    if cam.isRecording:
                        self.pub_scout_light.publish(True)  # only turn on scouting light when camera starts recording
                        rclpy.loginfo(
                            f'SW2312: Camera Manager: Scouting light turn on')
                    else:
                        self.pub_scout_light.publish(False)  # turn off scouting light when camera stops recording
                        rclpy.loginfo(
                            f'SW2312: Camera Manager: Scouting light turn off')

            if not cams:
                rclpy.loginfo(f'SW2312: Camera Manager: No camera settings in the config file')

        return return_msg



class Camera:
    def __init__(self, camType, mode, location, serviceName):
        self.camType = camType
        self.mode = mode
        self.isOpen = False
        self.isRecording = False
        self.location = location

        self.cameraRecordClient = cameraRecordClient(command=0, recording_basename='', serviceName=serviceName)

        
    def toggleOpen(self):

        response = CameraRecord.Response ()

        serviceState = self.cameraRecordClient.checkForService()
        if serviceState:

            self.cameraRecordClient.command = 1 if self.isOpen else 0
            response = self.cameraRecordClient.sendCameraCommand()

            if response.responseCode:
                self.isOpen = not self.isOpen

        else:
            rclpy.loginfo('SW2312: CameraManager - Unable to make request - toggle camera open state')
            rclpy.loginfo(
                'SW2312: CameraManager - ROS service ' + self.cameraRecordClient.serviceName + ' is not available')

        return response


    def toggleRecording(self, recording_basename):

        response = CameraRecord.Response ()

        serviceState = self.cameraRecordClient.checkForService()
        if serviceState:

            self.cameraRecordClient.command = 3 if self.isRecording else 2
            self.cameraRecordClient.recording_basename = recording_basename

            response = self.cameraRecordClient.sendCameraCommand()

            if response.responseCode:
                self.isRecording = not self.isRecording

        else:
            rclpy.loginfo('SW2312: CameraManager - Unable to make request - toggle camera recording state')
            rclpy.loginfo(
                'SW2312: CameraManager - ROS service ' + self.cameraRecordClient.serviceName + ' is not available')

        return response

    

######################################################################################################
## Main
######################################################################################################

def main(args):

    rclpy.init_node('cameraManager', anonymous=False)
    camManager = cameraManager()

    rate = rclpy.Rate(10) # 10hz

    # Due to rospy only allowing nodes to be called from within the main thread, we need to move them into here
    while not rclpy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)

