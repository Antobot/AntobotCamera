#realsense_camera.py
from __future__ import annotations
import numpy as np
import pyrealsense2 as rs
import cv2
import time


class CameraDriver:
    """
    Owns the RealSense pipeline and yields color+depth aligned frames.
    Not responsible for writing to disk.
    """
    def __init__(self, port, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.dims = (width, height)
        self.fps = fps
        self.port = port  # e.g. "/usb2/2-3/2-3:1.0"
        self.pipeline = None
        self.profile = None
        self.align = None
        self.depth_scale = 0.001
        self.color_intrinsics = None
        self._running = False
        self.last_color_frame = None
        self.selected_device = None
        self._last_roi_update_time = None

    
    def read_request_array(self):
        """
        Callback to read the latest frame as an array.

        Returns:
            out ( array | None ) : numpy array of image or None if no request available
        """
        if self.last_color_frame is not None:
            # COPY HERE: Ensures the network stream owns its own data
            # and won't crash if the camera loop overwrites the memory.
            return self.last_color_frame.copy() 
        return None
    
    def _select_device_by_port(self):
        #select device by port
        ctx = rs.context()
        self.selected_device = None
        for d in ctx.query_devices():
            port = d.get_info(rs.camera_info.physical_port)
            if self._port_core(port) == self.port:
                self.selected_device = d
                return self.selected_device
        return None

    def _port_core(self, physical_port: str) -> str:
        # Extract the substring between '/usb' and '/video4linux'
        # e.g. '/sys/.../usb2/2-3/2-3:1.0/video4linux/video2' -> '/usb2/2-3/2-3:1.0'
        try:
            usb_idx = physical_port.index("/usb")
            v4l_idx = physical_port.index("/video4linux")
            return physical_port[usb_idx:v4l_idx]
        except ValueError:
            # If format unexpected, just return the full path as a safe fallback
            return physical_port
        

    def _configure_sensors(self,):
        """Applies specialized configurations (like low-light settings) to the sensors."""
        
        # --- Tuned Parameters ---
        AUTO_EXPOSURE_LIMIT = 4000.0  # us
        AUTO_GAIN_LIMIT = 30.0
        WHITE_BALANCE_KELVIN = 4600  
        # -----------------------------
        
        try:
            for s in self.selected_device.query_sensors():
                # # DISABLE Global Time (for Multi-Cam stability)
                # # When enabled, the driver drops frames if timestamps don't match perfectly.
                # if s.supports(rs.option.global_time_enabled):
                #     s.set_option(rs.option.global_time_enabled, 0)
                
                # # DISABLE Auto-Exposure Priority
                # #    When ON, the camera lowers FPS in dark scenes to get more light.
                # #    We need constant FPS to prevent the 'Feeder' from timing out.
                # if s.supports(rs.option.auto_exposure_priority):
                #     s.set_option(rs.option.auto_exposure_priority, 0)

                # Global: Increase internal driver queue
                if s.supports(rs.option.frames_queue_size):
                    s.set_option(rs.option.frames_queue_size, 32)
                
                # Color Sensor Adjustments (Night Vision/Low-Light)
                    
                # 1. Ensure Auto-Exposure is ENABLED
                if s.supports(rs.option.enable_auto_exposure):
                    s.set_option(rs.option.enable_auto_exposure, 1)
                
                # 2. Set the Auto Exposure Limit (in microseconds)
                if s.supports(rs.option.auto_exposure_limit):
                    max_limit = s.get_option_range(rs.option.auto_exposure_limit).max
                    target_limit = min(max_limit, AUTO_EXPOSURE_LIMIT)
                    s.set_option(rs.option.auto_exposure_limit, target_limit)

                # 3. Set the Auto Gain Limit
                if s.supports(rs.option.auto_gain_limit):
                    max_gain = s.get_option_range(rs.option.auto_gain_limit).max
                    target_gain = min(max_gain, AUTO_GAIN_LIMIT)
                    s.set_option(rs.option.auto_gain_limit, target_gain)
                '''
                # 4. Optional: Enable Backlight Compensation
                if s.supports(rs.option.backlight_compensation):
                    s.set_option(rs.option.backlight_compensation, 1)

                # 5. White Balance (Manual)
                    # First, disable auto white balance
                    if s.supports(rs.option.enable_auto_white_balance):
                        s.set_option(rs.option.enable_auto_white_balance, 0)
                    
                    # Then, set the specific Kelvin value
                    if s.supports(rs.option.white_balance):
                        s.set_option(rs.option.white_balance, WHITE_BALANCE_KELVIN)
                '''
        except Exception as e:
            print(f"Warning: Failed to set sensor options: {e}")
    


    def set_col_roi(self, depth_u16: np.ndarray, max_depth_m: int = 0.6, buffer: int = 50) -> tuple[int, int]:
        """
        Calculates the left and right column indices for an ROI based on a maximum depth threshold.

        The ROI will encompass all pixels that are closer than max_depth_mm.

        Args:
            depth_u16: The (H, W) numpy array of depth values.
            max_depth_m: The maximum depth (in meters) to consider part of the ROI.
            buffer: Number of columns to add as padding/margin left and right of the detected region.

        Returns:
            A tuple (roi_left_idx, roi_right_idx) representing the bounding columns.
        """
        if depth_u16.ndim != 2:
            raise ValueError("depth_u16 must be a 2D array (H, W).")

        H, W = depth_u16.shape
        
        
        # 1. Define the downsampling scale (e.g., 4x downsampling)
        resize_scale = 4

        # 2. Adjust the threshold. Since we are looking for valid columns, the threshold 
        #    should be based on the image Height (H).
        #    We use half the height (H/2) as the effective threshold for a column to be 'valid'.
        PIXEL_THRESHOLD = H / 2
        threshold_scaled = PIXEL_THRESHOLD / resize_scale 

        # 3. Downsample using slicing (faster than cv2.resize, no interpolation needed)
        #    This view is a near-zero cost reference to the original array.
        depth_small = depth_u16[::resize_scale, ::resize_scale]

        # 4. Calculate Mask and Sum on the smaller image.
        #    The depth scale factor (0.0001) is assumed from the original code's context.
        depth_threshold_val = int(max_depth_m / self.depth_scale)
        mask_small = (depth_small > 0) & (depth_small <= depth_threshold_val)
        
        # Sum along axis=0 to get the count of valid pixels for each column.
        counts = np.sum(mask_small, axis=0) 

        # 5. Find the indices of valid columns.
        #    A column is valid if its count meets or exceeds the scaled height threshold.
        valid_cols = np.where(counts >= threshold_scaled)[0]

        if valid_cols.size > 0:
            # 6. Map the indices back to the original image coordinates (by multiplying by scale).
            tight_left = valid_cols.min() * resize_scale
            tight_right = valid_cols.max() * resize_scale
            
            # Apply the padding/margin buffer.
            # Ensure indices stay within the image bounds [0, W-1].
            roi_left_idx = max(0, tight_left - buffer)
            roi_right_idx = min(W - 1, tight_right + buffer)
        else:
            # If no depth data is valid, select the entire width.
            roi_left_idx = 0
            roi_right_idx = W - 1


        # --- RealSense API Application ---
        # This section would apply the calculated column ROI (min_x, max_x) 
        # to the selected device's sensor.
        for s in self.selected_device.query_sensors():
            if hasattr(s, 'as_roi_sensor'):
                try:
                    roi_sensor = s.as_roi_sensor() 
                    roi = rs.region_of_interest()
                    
                    # Apply the calculated column ROI (left/right)
                    roi.min_x = roi_left_idx
                    roi.max_x = roi_right_idx
                    # Set Y-axis (rows) to encompass the whole height.
                    roi.min_y = 0
                    roi.max_y = H - 1
                    roi_sensor.set_region_of_interest(roi)
                except Exception as e:
                    print(f"Warning: Failed to set column ROI on sensor: {e}") 
                    
            
        return roi_left_idx, roi_right_idx

    
    # def hardware_reset(self):
    #     """Forces a hardware reset on the device."""
    #     print("LEVEL 2 RECOVERY: TRIGGERING HARDWARE RESET")
    #     try:
    #         if self.selected_device:
    #             self.selected_device.hardware_reset()
    #             self.selected_device = None 
    #     except Exception as e:
    #         print(f"Hardware reset failed (device might be gone): {e}")
        
    #     # Wait for USB enumeration
    #     print("Waiting 5 seconds for device re-enumeration...")
    #     time.sleep(5)

    def soft_restart(self):
        """Attempts to stop and start the pipeline (Software Reset)."""
        print("LEVEL 1 RECOVERY: Attempting Software Restart ")
        self.stop()
        time.sleep(0.5) # Give it a moment to release resources
        self.start()    # This might fail if device is truly hung

    def start(self):
        if self._running:
            return
        
        # 1. Find the device
        self.selected_device = self._select_device_by_port()
        if self.selected_device is None:
            raise RuntimeError(f"CameraDriver: No RealSense device found at port {self.port}")

        # 2. Increase queue size to tolerate writer bursts
        try:
            for s in self.selected_device.query_sensors():
                # Set the internal driver queue (default is 16)
                if s.supports(rs.option.frames_queue_size):
                    s.set_option(rs.option.frames_queue_size, 32)

            self.align = rs.align(rs.stream.color)
        except Exception as e:
            print(f"Warning: Failed to set sensor options: {e}")

        # 3. Get Serial for Config
        serial = self.selected_device.get_info(rs.camera_info.serial_number)

        # 4. Configure Pipeline
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        
        # 5. Start Pipeline with a large Python buffer
        self.frame_queue = rs.frame_queue(150, keep_frames=True)
        self.profile = self.pipeline.start(cfg, self.frame_queue) 
        self._configure_sensors() 

        # 6. Setup Align
        self.align = rs.align(rs.stream.color)

        # 7. Get Intrinsics / Scale
        self.depth_scale = self.selected_device.first_depth_sensor().get_depth_scale()

        # Color intrinsics
        cstream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = cstream.get_intrinsics()
        self.color_intrinsics = dict(
            width=intr.width, height=intr.height,
            ppx=intr.ppx, ppy=intr.ppy, fx=intr.fx, fy=intr.fy,
            model=int(intr.model), coeffs=list(intr.coeffs),
        )

        # SANITY CHECK - Verify that frames are arriving
        try:
            _ = fs = self.frame_queue.wait_for_frame(1000).as_frameset()
            print(f"[{self.port}] Camera started and verified successfully.")
        except RuntimeError:
            raise RuntimeError(" CameraDriver: {self.port} No frames received from camera after starting pipeline.")
            
        self._running = True

    def stop(self):
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None
        self.profile = None
        self.align = None
        self._running = False

    def frames(self, timeout_ms=1000):
        """
        Generator yielding (color_bgr, depth_u16) where:
        - color_bgr: (H,W,3) uint8 BGR
        - depth_u16: (H,W) uint16 (Z16)
        """
        assert self._running, "CameraDriver not started."

        while self._running:
            fs = self.frame_queue.wait_for_frame(timeout_ms).as_frameset()

            aligned_frames = self.align.process(fs) 

            c = aligned_frames.get_color_frame()
            d = aligned_frames.get_depth_frame()
            
            if not c or not d:
                continue

            color_rgb = np.asarray(c.get_data())        # RGB
            color_bgr = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
            self.last_color_frame = color_bgr
            depth_u16 = np.asarray(d.get_data())        # Z16

            # update ROI every 3 seconds
            
            current_time = time.time()
            if self._last_roi_update_time is None or (current_time - self._last_roi_update_time > 3):
                roi_upper, roi_lower = self.set_col_roi(depth_u16, max_depth_m=0.6, buffer=50)
                self._last_roi_update_time = current_time
           
            yield color_bgr, depth_u16
