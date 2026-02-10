import json
import os, threading, queue
import numpy as np

#PyAV for MKV 
import av


class RgbdMkvWriter:
    """
    Write synchronized RGB (8-bit) and Depth (uint16) frames into a single MKV file.
    """
    def __init__(self, path, width, height, fps,
                 rgb_codec="h264_nvmpi", depth_codec="ffv1"):
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps
        self.rgb_codec = rgb_codec
        self.depth_codec = depth_codec

        self.container = None
        self.rgb_stream = None
        self.depth_stream = None
        self._pts = 0

    def open(self, *, depth_scale: float = None, color_intrinsics: dict = None, aligned=True):
        self.container = av.open(self.path, mode="w")

        # Metadata
        meta = {}
        if aligned is not None: meta["aligned"] = "true" if aligned else "false"
        if depth_scale is not None: meta["depth_scale"] = str(depth_scale)
        if color_intrinsics is not None: meta["color_intrinsics_json"] = json.dumps(color_intrinsics)
        if meta:
            self.container.metadata.update(meta)

        # RGB Stream
        self.rgb_stream = self.container.add_stream(self.rgb_codec, rate=self.fps)
        self.rgb_stream.width = self.width
        self.rgb_stream.height = self.height
        self.rgb_stream.pix_fmt = "yuv420p"
        
        # --- Encoder Settings ---
        if self.rgb_codec == "libx264":
            # Laptop / CPU Software Encoding Settings
            self.rgb_stream.options = {
                "preset": "ultrafast", 
                "tune": "zerolatency",
                "crf": "23"  # Constant Rate Factor (Lower is better quality, 23 is standard)
            }
        elif self.rgb_codec == "h264_nvmpi":
            # Jetson Hardware Encoder Options
            self.rgb_stream.bit_rate = 8_000_000 
            self.rgb_stream.options = {
                "preset": "medium", 
                "rc": "vbr",
                "maxrate": "8000000",
                "bufsize": "16000000",
                "g": "30"
            }

        # Depth Stream
        self.depth_stream = self.container.add_stream(self.depth_codec, rate=self.fps)
        self.depth_stream.width = self.width
        self.depth_stream.height = self.height
        self.depth_stream.pix_fmt = "gray16le"
        self.depth_stream.codec_context.gop_size = 1


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
        out_basename: str = "",
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        *,
        frames: int = 0,                     # 0 = unlimited             
    ):
        self.out_base = os.path.splitext(out_basename)[0]
        self.width, self.height, self.fps = width, height, fps
        self.frames_limit = int(frames)

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
    
    def open_writer(self, depth_scale=0.0001, color_intrinsics=None):
        self.mkv_writer.open(
            depth_scale=depth_scale,
            color_intrinsics=color_intrinsics,
            aligned=True
        )
    
    def push_frame(self, color_bgr, depth_u16):
        """
        External API to feed frames into the recording queue.
        Non-blocking (drops frame if writer is too slow).
        """
        if not self._running or self._stop.is_set():
            return

        try:
            self._frame_queue.put_nowait((color_bgr, depth_u16))
        except queue.Full:
            pass

    def _writer_loop(self):
        try:
            while not self._stop.is_set():
                try:
                    item = self._frame_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                if item is None: 
                    self._frame_queue.task_done()
                    break

                color_bgr, depth_u16 = item
                
                try:
                    self.mkv_writer.write(color_bgr, depth_u16)
                    self._frames_written += 1
                    
                    if self.frames_limit > 0 and self._frames_written >= self.frames_limit:
                        print(f"[Recorder] Limit reached ({self.frames_limit}). Stopping.")
                        self._stop.set()
                        
                except Exception as e:
                    print(f"[Recorder Error] Write failed: {e}")
                    self._stop.set()
                    break
                finally:
                    self._frame_queue.task_done()

        except Exception as e:
            print(f"[Recorder Crash] Loop failed: {e}")
        finally:
            self.mkv_writer.close()
            print(f"[Recorder] Finished. Saved: {self.mkv_path} ({self._frames_written} frames)")


    def start(self):
        if self._running:
            return

        self._stop.clear()
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=False)
        self._writer_thread.start()
        self._running = True


    def stop(self):
        if not self._running:
            return 0

        self._stop.set()        

        if self._writer_thread:
            self._writer_thread.join()
        
        self._running = False
        return self.frames_written
