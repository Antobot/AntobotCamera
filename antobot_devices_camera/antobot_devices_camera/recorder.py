from __future__ import annotations
import cv2, json, h5py
import os, threading, queue, time
import numpy as np

class DepthH5Writer:
    def __init__(self, path, h, w, chunk_t=16, compression="lzf", cache_mb=256):
        self.path = path
        self.h, self.w = h, w
        self.chunk_t = chunk_t
        self.compression = None if compression == "none" else compression
        self.cache_bytes = int(cache_mb * 1024 * 1024)
        self.f = None
        self.depth = None
        self._size = 0

    @property
    def size(self):
        return self._size

    def open(self):
        self.f = h5py.File(
            self.path, "w", libver="latest",
            rdcc_nbytes=self.cache_bytes, rdcc_nslots=1_000_003, rdcc_w0=0.75
        )
        self.depth = self.f.create_dataset(
            "depth", shape=(0, self.h, self.w), maxshape=(None, self.h, self.w),
            dtype="uint16", chunks=(self.chunk_t, self.h, self.w),
            compression=self.compression, track_times=False
        )
        try:  # per-dataset cache (may be ignored depending on backend)
            self.depth.id.set_chunk_cache(1_000_003, self.cache_bytes//2, 0.75)
        except Exception:
            pass

    def _ensure(self, need):
        if need > self.depth.shape[0]:
            grow = max(need, self.depth.shape[0] + max(self.chunk_t, 2048))
            self.depth.resize((grow, self.h, self.w))

    def write_batch(self, depths_u16: np.ndarray):
        """
        depths_u16: (T,H,W) uint16
        """
        T = depths_u16.shape[0]
        end = self._size + T
        self._ensure(end)
        self.depth[self._size:end] = depths_u16
        self._size = end

    def finalize_attrs(self, *, depth_scale: float, fps: float, color_intrinsics: dict):
        self.depth.attrs["depth_scale"] = np.float32(depth_scale)
        self.depth.attrs["fps"] = np.float32(fps)
        self.depth.attrs["color_intrinsics_json"] = json.dumps(color_intrinsics)

    def trim_and_close(self):
        if self._size < self.depth.shape[0]:
            self.depth.resize((self._size, self.h, self.w))
        try: self.f.flush()
        except: pass
        try: self.f.close()
        except: pass


class RgbMp4Writer:
    def __init__(self, path, width, height, fps):
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps
        self.vw = None

    def open(self):
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.vw = cv2.VideoWriter(self.path, fourcc, self.fps, (self.width, self.height))
        if not self.vw.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter: {self.path}")

    def write(self, bgr_frame):
        self.vw.write(bgr_frame)

    def close(self):
        if self.vw:
            self.vw.release()
            self.vw = None


class Recorder:
    """
    Pull frames from a CameraDriver, batch them, and write using DepthH5Writer (+optional MP4).
    Exposes start()/stop(), thread-safe. Designed to be called by a ROS2 node.
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
        h5_compression: str = "lzf",         # 'lzf', 'gzip', or 'none'
        chunk_t: int = 16,                   # temporal chunk depth
        batch: int = 16,                     # number of frames per batch write
        grow_step: int = 2048,               # how much to grow dataset when full
        cache_mb: int = 256,                 # HDF5 chunk cache size
    ):
        self.cam = camera_driver
        self.out_base = os.path.splitext(out_basename)[0]
        self.width, self.height, self.fps = width, height, fps
        self.frames_limit = int(frames)
        self.chunk_t = int(chunk_t)
        self.batch = int(batch)
        self.grow_step = int(grow_step)
        self.cache_mb = int(cache_mb)

        self.h5_path = self.out_base + ".h5"
        self.mp4_path = self.out_base + ".mp4"

        self.depth_writer = DepthH5Writer(
            self.h5_path, h=height, w=width, chunk_t=self.chunk_t,
            compression=h5_compression, cache_mb=self.cache_mb
        )
        self.rgb_writer = RgbMp4Writer(self.mp4_path, width, height, fps)

        self._frame_queue = queue.Queue(maxsize=8)  # bounded backpressure
        self._writer_thread = None
        self._stop = threading.Event()
        self._frames_written = 0
        self._running = False



    @property
    def frames_written(self):
        return self._frames_written

    def _writer_loop(self):
        # Pre-open writers
        self.depth_writer.open()
        self.rgb_writer.open()

        # Preallocation for batched writes
        batch_T = self.batch
        depths = np.empty((batch_T, self.height, self.width), dtype=np.uint16)
        bi = 0
        last_flush = time.time()

        try:
            while not self._stop.is_set():
                try:
                    item = self._frame_queue.get(timeout=0.2)
                except queue.Empty:
                    item = None

                if item is None:
                    # drain/flush if stopping
                    if self._stop.is_set():
                        break
                    continue

                color_bgr, depth_u16 = item

                # write RGB immediately (frame-by-frame is fine)
                if self.rgb_writer:
                    self.rgb_writer.write(color_bgr)

                # fill batch for depth
                depths[bi] = depth_u16
                bi += 1

                if bi == batch_T:
                    self.depth_writer.write_batch(depths)
                    self._frames_written += batch_T
                    bi = 0

                    # periodic flush
                    now = time.time()
                    if now - last_flush > 10.0:
                        try: self.depth_writer.f.flush()
                        except: pass
                        last_flush = now

                self._frame_queue.task_done()
        finally:
            # write residuals
            if bi > 0:
                self.depth_writer.write_batch(depths[:bi])
                self._frames_written += bi

            # finalize attrs
            self.depth_writer.finalize_attrs(
                depth_scale=self.cam.depth_scale,
                fps=float(self.fps),
                color_intrinsics=self.cam.color_intrinsics,
            )
            # close files
            self.depth_writer.trim_and_close()
            if self.rgb_writer:
                self.rgb_writer.close()
                print(f"Recorded RGB video to: {self.mp4_path}")

    def start(self):
        if self._running:
            print("[WARN] Recorder already running, ignoring start()")
            return
        self.cam.start()   # ensure camera is up
        self._stop.clear()
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=False)
        self._writer_thread.start()
        self._running = True

        # feeder loop on a helper thread to avoid blocking ROS spin
        self._feeder = threading.Thread(target=self._feeder_loop, daemon=False)
        self._feeder.start()

    def _feeder_loop(self):
        for color_bgr, depth_u16 in self.cam.frames():
            if self._stop.is_set():
                break
            # bounded put with drop-old strategy if full
            try:
                self._frame_queue.put((color_bgr, depth_u16), timeout=0.05)
            except queue.Full:
                # drop 1 oldest to keep latency bounded
                try: _ = self._frame_queue.get_nowait(); self._frame_queue.task_done()
                except queue.Empty: pass
                try: self._frame_queue.put_nowait((color_bgr, depth_u16))
                except queue.Full: pass

            if self.frames_limit > 0 and self.frames_written >= self.frames_limit:
                break

        self._stop.set()

    def stop(self):
        if not self._running:
            return 0
        self._stop.set()
        # wait writers to finish
        if self._writer_thread:
            self._writer_thread.join() 
        if self._feeder:
            self._feeder.join()
        self.cam.stop()
        self._running = False
        return self.frames_written
    
