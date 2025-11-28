from realsense_camera import CameraDriver
from preview_streamer import PreviewStreamer
import threading

# create camera driver
cam1 = CameraDriver(port="/usb2/2-2/2-2:1.0", width=1280, height=720, fps=30)
cam1.start()

# loop to fetch frames from camera
def frame_loop(cam):
    for color_bgr, depth_u16 in cam.frames():
        pass

t = threading.Thread(target=frame_loop, args=(cam1,), daemon=True)
t.start()

# create preview streamer with camera streams
track_dict = {
    "cam1": cam1.stream_track,
    "cam2": None
}
streamer = PreviewStreamer(track_dict)
# start preview streamer
streamer.run()
