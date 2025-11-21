import json
import os, threading, queue
import numpy as np

#PyAV for MKV 
import av


class RgbdMkvWriter:
    """
    Write synchronized RGB (8-bit) and Depth (uint16) frames into a single MKV file:
      - RGB: libx264, yuv420p
      - Depth: ffv1, gray16le (lossless)
    """
    def __init__(self, path, width, height, fps,
                 rgb_codec="h264_nvmpi", depth_codec="ffv1",
                 x264_preset="veryfast", x264_crf=18):
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps
        self.rgb_codec = rgb_codec
        self.depth_codec = depth_codec
        self.x264_preset = x264_preset
        self.x264_crf = x264_crf

        self.container = None
        self.rgb_stream = None
        self.depth_stream = None
        self._pts = 0  # 1 tick per frame, since time_base = 1/fps

    def open(self, *, depth_scale: float = None, color_intrinsics: dict = None, aligned=True):
        self.container = av.open(self.path, mode="w")

        # Global/container metadata (optional)
        meta = {}
        if aligned is not None: meta["aligned"] = "true" if aligned else "false"
        # Store scale/intrinsics at the container level; also copy to depth stream
        if depth_scale is not None: meta["depth_scale"] = str(depth_scale)
        if color_intrinsics is not None: meta["color_intrinsics_json"] = json.dumps(color_intrinsics)
        if meta:
            self.container.metadata.update(meta)

        # RGB stream (H.264)
        self.rgb_stream = self.container.add_stream(self.rgb_codec, rate=self.fps)
        self.rgb_stream.width = self.width
        self.rgb_stream.height = self.height
        self.rgb_stream.pix_fmt = "yuv420p"
        # Encoder tuning
        # Jetson nvmpi encoder (jetson-ffmpeg)
        self.rgb_stream.bit_rate = 8_000_000  # 8 Mbps; tune as needed
        # These map roughly to typical ffmpeg CLI usage:
        #   -c:v h264_nvmpi -preset medium -rc vbr -b:v 8M -maxrate 8M -bufsize 16M -g 30
        self.rgb_stream.options = {
            "preset": "medium",      # or "fast", "slow" depending on what your build supports
            "rc": "vbr",             # rate control: vbr or cbr
            "maxrate": "8000000",    # match bit_rate
            "bufsize": "16000000",   # 2x bit_rate is a common starting point
            "g": "30",               # GOP size (keyframe every 30 frames at 30 FPS = 1s)
        }

        # Depth stream (FFV1 lossless)
        self.depth_stream = self.container.add_stream(self.depth_codec, rate=self.fps)
        self.depth_stream.width = self.width
        self.depth_stream.height = self.height
        # FFV1 expects gray16le for 16-bit single channel
        self.depth_stream.pix_fmt = "gray16le"
        # Intra-only style; FFV1 is inherently intra, but g=1 keeps keyframes each frame
        self.depth_stream.codec_context.gop_size = 1
        # Put depth-specific metadata on the depth stream too
        if depth_scale is not None:
            self.depth_stream.metadata["depth_scale"] = str(depth_scale)
        if color_intrinsics is not None:
            self.depth_stream.metadata["color_intrinsics_json"] = json.dumps(color_intrinsics)
        self.depth_stream.metadata["stream"] = "depth_gray16le"


    def write(self, bgr_frame: np.ndarray, depth_u16: np.ndarray):
        """
        Accepts:
          - bgr_frame: (H,W,3) uint8, BGR
          - depth_u16: (H,W)   uint16, millimeters or similar units
        """
        # RGB: convert BGR->YUV420P through PyAV frames
        f_rgb = av.VideoFrame.from_ndarray(bgr_frame, format="bgr24")
        f_rgb = f_rgb.reformat(self.width, self.height, "yuv420p")
        f_rgb.pts = self._pts

        # Depth: 16-bit grayscale
        f_d = av.VideoFrame.from_ndarray(depth_u16, format="gray16le")
        # ensure same size
        if f_d.width != self.width or f_d.height != self.height:
            f_d = f_d.reformat(self.width, self.height, "gray16le")
        f_d.pts = self._pts

        # Encode & mux packets per stream
        for pkt in self.rgb_stream.encode(f_rgb):
            self.container.mux(pkt)
        for pkt in self.depth_stream.encode(f_d):
            self.container.mux(pkt)

        self._pts += 1

    def close(self):
        # Flush encoders
        for pkt in self.rgb_stream.encode():
            self.container.mux(pkt)
        for pkt in self.depth_stream.encode():
            self.container.mux(pkt)
        self.container.close()


class Recorder:
    """
    Pull frames from a CameraDriver, and write:
      - MKV (RGB+Depth) with RgbdMkvWriter 
    """
    def __init__(
        self,
        camera_driver,
        *,
        out_basename: str = "recordings/session",
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        frames: int = 0,                     # 0 = unlimited
        container: str = "mkv"               
    ):
        self.cam = camera_driver
        self.out_base = os.path.splitext(out_basename)[0]
        self.width, self.height, self.fps = width, height, fps
        self.frames_limit = int(frames)
        self.container_kind = container

        self._frame_queue = queue.Queue(maxsize=8)  # bounded backpressure
        self._writer_thread = None
        self._stop = threading.Event()
        self._frames_written = 0
        self._running = False

        # Writer
        self.mkv_writer = None
        self.mkv_path = self.out_base + ".mkv"
        self.mkv_writer = RgbdMkvWriter(self.mkv_path, width, height, fps)

    @property
    def frames_written(self):
        return self._frames_written

    def _writer_loop(self):
        try:
            while True:

                try:
                    item = self._frame_queue.get(timeout=1.0)
                except queue.Empty:
                    if self._stop.is_set():
                        break
                    continue

                if item is None:
                    self._frame_queue.task_done()
                    break 

                color_bgr, depth_u16 = item

                try:
                    # Catch Disk Full / IO Errors
                    self.mkv_writer.write(color_bgr, depth_u16)
                    self._frames_written += 1
                except Exception as e:
                    self._error = e
                    print(f"[Recorder Error] Write failed (Disk full?): {e}")
                    self._stop.set() # Stop the feeder immediately
                    self._frame_queue.task_done()
                    break

        except Exception as e:
            # Catch anything else unexpected
            self._error = e
            print(f"[Recorder Error] Writer crashed unexpectedly: {e}")

        finally:
            self.mkv_writer.close()
            print(f"Recorded RGB-D MKV to: {self.mkv_path}")

    def start(self):
        if self._running:
            print("[WARN] Recorder already running, ignoring start()")
            return
        self.cam.start()
        self._stop.clear()

        # Pre-open MKV with metadata from camera
        self.mkv_writer.open(
            depth_scale=self.cam.depth_scale,
            color_intrinsics=self.cam.color_intrinsics,
            aligned=True
        )

        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=False)
        self._writer_thread.start()
        self._running = True

        self._feeder = threading.Thread(target=self._feeder_loop, daemon=False)
        self._feeder.start()

    def _feeder_loop(self):
        try:
            for color_bgr, depth_u16 in self.cam.frames():

                if self._stop.is_set():
                    break
                
                try:
                    self._frame_queue.put((color_bgr, depth_u16), block=False)
                except queue.Full: # Queue is full. The writer is slow.

                    try:
                        _ = self._frame_queue.get_nowait() # Remove the oldest frame to make space
                        self._frame_queue.task_done()
                    except queue.Empty:
                        # This happens if the writer cleared a slot at the exact same moment.
                        # We ignore it because it means we have space now.
                        pass
                    
                    # put the new frame in the space we just made
                    try:
                        self._frame_queue.put_nowait((color_bgr, depth_u16))
                    except queue.Full:
                        # If it's STILL full (extremely rare race condition), 
                        # we just drop the current frame to prevent crashing.
                        pass 

                if self.frames_limit > 0 and self._frames_written >= self.frames_limit:
                    self._stop.set()
                    break

        except Exception as e:
            self._error = e
            print(f"[Recorder Error] Feeder crashed: {e}")
            self._stop.set()
        finally:
            # Signal the writer to stop
            self._frame_queue.put(None)


    def stop(self):
            if not self._running:
                return 0

            self._stop.set()

            if self._feeder:
                self._feeder.join()
        

            if self._writer_thread:
                self._writer_thread.join()
            
            self.cam.stop()
            
            self._running = False
            return self.frames_written
