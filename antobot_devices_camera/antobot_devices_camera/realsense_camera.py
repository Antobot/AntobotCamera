#realsense_camera.py
from __future__ import annotations
import numpy as np
import pyrealsense2 as rs
import cv2

class CameraDriver:
    """
    Owns the RealSense pipeline and yields color+depth aligned frames.
    Not responsible for writing to disk.
    """
    def __init__(self, port, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.port = port  # e.g. "/usb2/2-3/2-3:1.0"
        self.pipeline = None
        self.profile = None
        self.align = None
        self.depth_scale = 0.001
        self.color_intrinsics = None
        self._running = False

    def _select_device_by_port(self):
        #select device by port
        ctx = rs.context()
        selected_device = None
        for d in ctx.query_devices():
            port = d.get_info(rs.camera_info.physical_port)
            if self._port_core(port) == self.port:
                selected_device = d
                return selected_device
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

    def start(self):
        if self._running:
            return
        
        # 1. Find the device
        selected_device = self._select_device_by_port()
        if selected_device is None:
            raise RuntimeError(f"CameraDriver: No RealSense device found at port {self.port}")

        # 2. Increase queue size to tolerate writer bursts
        try:
            for s in selected_device.query_sensors():
                # Set the internal driver queue (default is 16)
                if s.supports(rs.option.frames_queue_size):
                    s.set_option(rs.option.frames_queue_size, 32)

            self.align = rs.align(rs.stream.color)
        except Exception as e:
            print(f"Warning: Failed to set sensor options: {e}")

        # 3. Get Serial for Config
        serial = selected_device.get_info(rs.camera_info.serial_number)

        # 4. Configure Pipeline
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        
        # 5. Start Pipeline with a large Python buffer
        self.frame_queue = rs.frame_queue(150, keep_frames=True)
        self.profile = self.pipeline.start(cfg, self.frame_queue) 

        # 6. Setup Align
        self.align = rs.align(rs.stream.color)

        # 7. Get Intrinsics / Scale
        self.depth_scale = selected_device.first_depth_sensor().get_depth_scale()

        # Color intrinsics
        cstream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = cstream.get_intrinsics()
        self.color_intrinsics = dict(
            width=intr.width, height=intr.height,
            ppx=intr.ppx, ppy=intr.ppy, fx=intr.fx, fy=intr.fy,
            model=int(intr.model), coeffs=list(intr.coeffs),
        )
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
            depth_u16 = np.asarray(d.get_data())        # Z16

            yield color_bgr, depth_u16
