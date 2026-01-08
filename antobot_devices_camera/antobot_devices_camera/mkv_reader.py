import av
import numpy as np

def read_rgbd_mkv_minimal(path):
    """
    Minimal reader for MKV recorded by RgbdMkvWriter.
    Yields (rgb_bgr8, depth_u16, t_seconds)
    """
    container = av.open(path)
    streams = [s for s in container.streams.video]
    if len(streams) != 2:
        raise RuntimeError("Expected 2 video streams (RGB + Depth)")

    # Identify which is which
    if "gray16le" in streams[0].codec_context.format.name.lower():
        depth_stream, rgb_stream = streams
    else:
        rgb_stream, depth_stream = streams

    tb = rgb_stream.time_base
    rgb_frames, depth_frames = {}, {}

    for packet in container.demux(streams):
        s = packet.stream  
        for frame in packet.decode():
            if frame.pts is None:
                continue
            if s == rgb_stream:
                rgb_frames[frame.pts] = frame.to_ndarray(format="bgr24")
            elif s == depth_stream:
                arr = frame.to_ndarray(format="gray16le")
                if arr.ndim == 3:
                    arr = arr[..., 0]
                depth_frames[frame.pts] = np.asarray(arr, dtype="<u2")

            # emit if both have same pts
            pts = frame.pts
            if pts in rgb_frames and pts in depth_frames:
                rgb = rgb_frames.pop(pts)
                depth = depth_frames.pop(pts)
                t = float(pts * tb)
                yield rgb, depth, t

    container.close()


for idx,(rgb, depth, t) in enumerate(read_rgbd_mkv_minimal("/root/ros2_ws/data/20251111/104555_l.mkv")):
    print(rgb.shape, depth.shape, t)

    if idx >= 5:
        break   



meta = dict(av.open("/root/ros2_ws/data/20251111/104555_l.mkv").metadata)
scale = float(meta.get("depth_scale", 0.001))