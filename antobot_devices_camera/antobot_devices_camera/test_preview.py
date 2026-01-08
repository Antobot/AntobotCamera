from realsense_camera import CameraDriver
from preview_streamer import PreviewStreamer, CameraStreamTrack
import threading

# Initialise Camera Driver
cam1 = CameraDriver(port="/usb2/2-1/2-1.3/2-1.3:1.0", width=1280, height=720, fps=30)
cam1.start()

cam_track = CameraStreamTrack(
    read_array_callback=cam1.read_request_array, 
    frame_dims=(cam1.height, cam1.width) 
)

# The background thread loops through frames() and updates last_color_frame
def frame_loop(cam):
    for color_bgr, depth_u16 in cam.frames():
        pass

t = threading.Thread(target=frame_loop, args=(cam1,), daemon=True)
t.start()

# Create streamer and start
track_dict = {
    "cam1": cam_track,
    "cam2": None
}
streamer = PreviewStreamer(track_dict)
# Run WebRTC server (block)
streamer.run()
