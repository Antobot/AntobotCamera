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
import rospy
import rospkg
from datetime import datetime
from std_msgs.msg import Bool, String


from antobot_devices_msgs.srv import camManager, camManagerResponse
from antobot_devices_msgs.srv import antoRec, antoRecResponse
# from antobot_manager_jobs.updateProgressClient import progressUpdateClient
from antoRecClient import antoRecClient



class cameraManager:
    def __init__(self):
        ##############################################################################
        ## Check for the simulation parameter which should be set by am_sim     
        ##############################################################################

        self.cameras = {}
        self.read_config_file()
        try:
            self.sim = rospy.get_param("/simulation")

        except:
            self.sim=False # If the simulation parameter has not been assigned, assume not a simulation


        ##############################################################################
        ## Run either real or simulated manager    
        ##############################################################################

        if self.sim:
            print('Camera Manager: This is a simulation - using fake camera calls.')
        else:
            print('Camera Manager: This is not a simulation - using real ZED2 camera commands.')


        # Get ros parameters for cameras
        #camera_config = rospy.get_param("/camera")

        # Create a service to allow other nodes to start/stop cameras
        self.srvCamMgr = rospy.Service("/antobot/camera_manager/camera", camManager, self._serviceCallbackCamMgr)

        # self.updateClient = progressUpdateClient(state=0, sourceID='camManager')
        
        self.pub_scout_light = rospy.Publisher("/antobot_manager_device/scout_light",Bool, queue_size=1)

    def read_config_file(self):
        rospack = rospkg.RosPack()

        try:
            path = rospack.get_path('antobot_description')
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
                rospy.loginfo(f'SW2312: Camera Manager: No camera settings found in config')

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
        return_msg = camManagerResponse()
        cams = None

        if request.command == 2:  # toggle camera open state
            if request.camera_num == 3:  # left camera
                cams = self.cameras['left']

            elif request.camera_num == 4: # right camera
                cams = self.cameras['right']

            for cam in cams.values():
                if cam:
                    rospy.loginfo(f'SW2312: Camera Manager: {cam.location} {cam.camType} camera open state: {cam.isOpen}')
                    rospy.loginfo(f'SW2312: Camera Manager: Make request to toggle {cam.location} {cam.camType} camera open state')
                    response = cam.toggleOpen()
                    return_msg.responseCode = response.responseCode
                    return_msg.responseString = response.responseString


        elif request.command == 3:  # toggle camera recording state
            if request.camera_num == 3:  # left camera
                cams = self.cameras['left']

            elif request.camera_num == 4:  # right camera
                cams = self.cameras['right']

            for cam in cams.values():
                rospy.loginfo(f'SW2312: Camera Manager: {cam.location} {cam.camType} camera recording state: {cam.isRecording}')
                rospy.loginfo(f'SW2312: Camera Manager: Make request to toggle {cam.location} {cam.camType} camera recording state')
                response = cam.toggleRecording()
                return_msg.responseCode = response.responseCode
                return_msg.responseString = response.responseString

                if cam.camType == 'zed':  # only check the zed status for now
                    if cam.isRecording:
                        self.pub_scout_light.publish(True)  # only turn on scouting light when camera starts recording
                        rospy.loginfo(
                            f'SW2312: Camera Manager: Scouting light turn on')
                    else:
                        self.pub_scout_light.publish(False)  # turn off scouting light when camera stops recording
                        rospy.loginfo(
                            f'SW2312: Camera Manager: Scouting light turn off')

                    if return_msg.responseCode:
                        self.updateClient.state = 1
                    else:
                        self.updateClient.state = 0

                    # Report any changes to job manager
                    self.updateJobManagerState()

            if not cams:
                rospy.loginfo(f'SW2312: Camera Manager: No camera settings in the config file')

        return return_msg


    # def updateJobManagerState(self):
    #     """ Update the camera state in jobManager """
# 
    #     serviceState=self.updateClient.checkForService()
    #     if serviceState: # If the service is available
    #         managerResponse = self.updateClient.sendProgressUpdate()
    #     else:
    #         print('UV Manager: Unable to make request')
    #         print('UV Manager: ROS service ' + self.updateClient.serviceName + ' is not available')
    #         # TODO - Add an exception


class Camera:
    def __init__(self, camType, mode, location, serviceName):
        self.camType = camType
        self.mode = mode
        self.isOpen = False
        self.isRecording = False
        self.location = location
        self.filename = None

        self.antoRecClient = antoRecClient(command=0, timestamp='', serviceName=serviceName)
        self.rec_manager_timestamp_sub = rospy.Subscriber("/recManager/timestamp",String,self.timestamp_callback)

    def timestamp_callback(self,data):
        self.filename = data.data
        return
        
    def toggleOpen(self):

        response = antoRecResponse()

        serviceState = self.antoRecClient.checkForService()
        if serviceState:

            timestamp = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
            self.antoRecClient.command = 1 if self.isOpen else 0
            self.antoRecClient.timestamp = timestamp

            response = self.antoRecClient.sendCameraCommand()

            if response.responseCode:
                self.isOpen = not self.isOpen

        else:
            rospy.loginfo('SW2312: CameraManager - Unable to make request - toggle camera open state')
            rospy.loginfo(
                'SW2312: CameraManager - ROS service ' + self.antoRecClient.serviceName + ' is not available')

        return response


    def toggleRecording(self):

        response = antoRecResponse()

        serviceState = self.antoRecClient.checkForService()
        if serviceState:

            timestamp = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
            self.antoRecClient.command = 3 if self.isRecording else 2
            if self.filename is not None:
                date_part,time_H,time_M,time_S=timestamp.rsplit('_',3)
                timestamp = f"{date_part}_{self.filename}"
            self.antoRecClient.timestamp = timestamp

            response = self.antoRecClient.sendCameraCommand()

            if response.responseCode:
                self.isRecording = not self.isRecording

        else:
            rospy.loginfo('SW2312: CameraManager - Unable to make request - toggle camera recording state')
            rospy.loginfo(
                'SW2312: CameraManager - ROS service ' + self.antoRecClient.serviceName + ' is not available')

        return response

    

######################################################################################################
## Main
######################################################################################################

def main(args):
    
    rospy.init_node('cameraManager', anonymous=False)
    camManager = cameraManager()

    rate = rospy.Rate(10) # 10hz

    # Due to rospy only allowing nodes to be called from within the main thread, we need to move them into here
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)

