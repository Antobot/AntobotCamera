#realsense_camera.py
from __future__ import annotations
import numpy as np
import pyrealsense2 as rs

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

        selected_device = self._select_device_by_port()
        if selected_device is None:
            raise RuntimeError(f"CameraDriver: No RealSense device found at port {self.port}")
        
        serial = selected_device.get_info(rs.camera_info.serial_number)

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.profile = self.pipeline.start(cfg)

        device = self.profile.get_device()
        # Increase queue size to tolerate writer bursts
        try:
            for s in device.query_sensors():
                if s.supports(rs.option.frames_queue_size):
                    s.set_option(rs.option.frames_queue_size, 16)

            self.align = rs.align(rs.stream.color)
        except Exception:
            pass

        # Depth scale
        self.depth_scale = device.first_depth_sensor().get_depth_scale()

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
        import cv2

        while self._running:
            fs = self.pipeline.wait_for_frames(timeout_ms)
            if self.align:
                fs = self.align.process(fs)

            c = fs.get_color_frame()
            d = fs.get_depth_frame()
            if not c or not d:
                continue

            color_rgb = np.asarray(c.get_data())        # RGB
            color_bgr = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
            depth_u16 = np.asarray(d.get_data())        # Z16

            yield color_bgr, depth_u16
