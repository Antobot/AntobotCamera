#!/usr/bin/env python3

# Copyright (c) 2025, ANTOBOT LTD.
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


import rclpy
from rclpy.node import Node
from antobot_camera_msgs.srv import CameraRecord


class CameraRecordClient(Node):
    """A class that handles a client to provide updates to higher level nodes"""

    def __init__(self, serviceName):
        super().__init__('camera_record_client')
        self.cli = self.create_client(CameraRecord, serviceName)
        self.serviceName = serviceName

    def check_service(self, wait=0.5):

        if not self.cli.wait_for_service(timeout_sec=wait):
            self.get_logger().info(f'service {self.serviceName} not available')
            return False
        return True


    def sendCameraCommand(self, command, recording_basename):

        if not self.check_service():
            return None

        req = CameraRecord.Request()
        req.command = command
        req.recording_basename = recording_basename

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)

        try:
            fut.result()
            return fut.result()
        except Exception as e:
            self.get_logger().info(f'Service call failed {e}')
            return None
        

def main():
    # Create the class to handle client-side interaction
    cameraRecordClient = CameraRecordClient(serviceName='/antobot_devices_camera/rpi/record/left')
    camManagerResponse = cameraRecordClient.sendCameraCommand(command=1, recording_basename='test')


if __name__ == "__main__":
    main()
