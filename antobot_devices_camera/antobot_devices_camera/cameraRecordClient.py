#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
#
# Description: 	A client for letting tasks connect to the camera manager, activating individual cameras.
#
# Contacts: 	jinhuan.liu@antobot.ai
#               william.eaton@antobot.ai
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


import rospy
import rosservice
from antobot_camera_msgs.srv import cameraRecord, cameraRecordResponse


class cameraRecordClient():
    """A class that handles a client to provide updates to higher level nodes"""

    def __init__(self, command, recording_basename, serviceName):

        self.serviceName = serviceName

        self.cameraRecordClient = rospy.ServiceProxy(self.serviceName, cameraRecord)
        self.command = command
        self.recording_basename = recording_basename

    def checkForService(self):
        service_list = rosservice.get_service_list()
        if self.serviceName in service_list:
            return True
        else:
            return False

    def sendCameraCommand(self):

        # In ROS it's common to wait for a service. However, this blocks execution and is not always useful. Use checkForService method instead.
        # rospy.wait_for_service('localUserInput')
        # camCommand = camManagerRequest
        # camCommand.camera_num=self.camera_num
        # camCommand.command=self.command

        try:
            response = self.cameraRecordClient(self.command, self.recording_basename)
            return response

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":

    # Create the class to handle client-side interaction
    cameraRecordClient = cameraRecordClient(command=2, recording_basename='2024_06_28_14_16_00', serviceName='/antobot/camera_record/left')

    # Check that the service is availble before trying to send requests
    serviceState = cameraRecordClient.checkForService()

    if serviceState:  # If the service is available
        camManagerResponse = cameraRecordClient.sendCameraCommand()
    else:
        print('Unable to make request')
        print('ROS service ' + cameraRecordClient.serviceName + ' is not available')
