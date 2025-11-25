from realsense_camera import CameraDriver
from preview_streamer import PreviewStreamer
import threading

# 创建相机
cam1 = CameraDriver(port="/usb2/2-2/2-2:1.0", width=1280, height=720, fps=30)
cam1.start()

# 后台线程循环 frames() 更新 last_color_frame
def frame_loop(cam):
    for color_bgr, depth_u16 in cam.frames():
        pass

t = threading.Thread(target=frame_loop, args=(cam1,), daemon=True)
t.start()

# 创建 streamer 并启动
track_dict = {
    "cam1": cam1.stream_track,
    "cam2": None
}
streamer = PreviewStreamer(track_dict)
# 启动 WebRTC 服务器 (阻塞)
streamer.run()
