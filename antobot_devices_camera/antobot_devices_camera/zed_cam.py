import cv2
import numpy as np
import pyzed.sl as sl
from datetime import time as t


def param_init():
    """
    Function to set parameters of the cameras.

    Returns:
        init_params (sl.InitParameters): parameters and runtime settings

    """
    init_params = sl.InitParameters()
    init_params.camera_fps = 30  # set frames per second
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    init_params.coordinate_units = sl.UNIT.CENTIMETER  # Use CENTIMETER units

    return init_params



class ZedCamera:
    def __init__(self, argv=None):
        """
        Initialises a new zed camera class object.

        Args:
            argv (sys.argv): system arguments

        """
        self.cam = sl.Camera()
        self.init_params = param_init()
        self.runtime = sl.RuntimeParameters()

        self.display_image = False


    def reboot(self):
        self.cam.reboot(0)
        self.cam = sl.Camera()
        self.init_params = param_init()
        self.runtime = sl.RuntimeParameters()


    def run_frame_capture(self):
        return self.cam.grab(self.runtime) == sl.ERROR_CODE.SUCCESS

    def adjust_exposure(self, datetime_obj):
        # todo: fix
        if not (t(17, 0) < datetime_obj.time() or datetime_obj.time() < t(6, 0)):
            # perform auto exposure every 5 seconds
            current_time = self.cam.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_seconds()
            if current_time - self.last_time > 5:
                left_img = sl.Mat()
                self.cam.retrieve_image(left_img, sl.VIEW.LEFT)
                self.set_cam_roi(left_img)
                self.last_time = self.cam.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_seconds()


    def open_camera(self):
        """
        This function opens the requested camera and display current view if asked

        Returns:
            status (sl.ERROR_CODE): If SUCCESS is returned, the camera is open. Every other code indicates an error.

        """

        status = self.cam.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))

        self.last_time = self.cam.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_seconds()

        return status == sl.ERROR_CODE.SUCCESS

    def close_camera(self):
        """
        This function close the requested camera. If recording hasn't been stopped, it will stop it firstly.

        """

        cv2.destroyAllWindows()
        self.cam.close()

    def start_recording(self, output_basename):

        """
        Function to enable record the footage from the zed camera.

        Returns:
            err (sl.ERROR_CODE): If SUCCESS is returned, recording is started. Every other code indicates an error.

        """

        recording_param = sl.RecordingParameters(f"{output_basename}.svo", sl.SVO_COMPRESSION_MODE.H265)
        err = self.cam.enable_recording(recording_param)
        if err != sl.ERROR_CODE.SUCCESS:
            return err == sl.ERROR_CODE.SUCCESS

        # todo: fix

        # if t(17, 0) < datetime_obj.time() or datetime_obj.time() < t(6, 0):
        #     self.cam.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 10)
        #     self.cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)
        #     self.cam.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, 5000)


        return err == sl.ERROR_CODE.SUCCESS

    def is_recording_started(self):

        return self.cam.get_recording_status().is_recording

    def is_open(self):

        return self.cam.is_opened()


    def stop_recording(self):
        """
        Function to stop recording.

        """
        self.cam.disable_recording()

        # self.cam.close()


    def set_cam_roi(self, left_img_mat):
        """
        Function to find dark region of current image and send this ROI to camera exposure settings.

        Args:
            left_img_mat (pyzed.sl.mat): sl.Mat class to store the left image data.

        """
        left_img_array = left_img_mat.get_data()
        left_img_gray = cv2.cvtColor(left_img_array, cv2.COLOR_BGR2GRAY)  # grayscale image
        # Utilizes the Otsu's Binarization to find the dark region(not a perfect rectangle, with noise)
        _, binary_img = cv2.threshold(left_img_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Return the rows which the total number of black pixels is greater than half of the width of the image
        mask = np.count_nonzero(binary_img, axis=1) < left_img_array.shape[0] / 2
        dark_idx = np.nonzero(mask)
        # find min, max index of the dark region
        try:
            min_dark_idx = max(np.min(dark_idx) - 5, 0)
        except:
            min_dark_idx = 0
        try:
            max_dark_idx = min(np.max(dark_idx) + 5, left_img_array.shape[0] - 1)
        except:
            max_dark_idx = left_img_array.shape[0] - 1
        # generate region of interest(ROI)
        roi = sl.Rect(0, min_dark_idx, left_img_array.shape[1], max_dark_idx - min_dark_idx)
        # sent ROI to zed camera settings
        self.cam.set_camera_settings_roi(sl.VIDEO_SETTINGS.AEC_AGC_ROI, roi, sl.SIDE.BOTH)

    def show_image(self, cam_name):
        """
        Display the left and right view images side by side.

        """
        img = sl.Mat()
        self.cam.retrieve_image(img, sl.VIEW.SIDE_BY_SIDE)

        img_np = img.get_data()
        cv2.imshow(cam_name, img_np)
        cv2.waitKey(10)