from antobot_devices_camera.rpi_insight_camera import RPiInsightCamera
import time
import numpy as np
import matplotlib.pyplot as plt
import json



def rec():
    cam = RPiInsightCamera(preview=False, raw=False, framerate=framerate)

    cam.open_camera()
    cam.start_recording('/home/ant/catkin_ws/src/scoutRecord/AntobotDevices/AntobotCamera/antobot_devices_camera/test/temp')

    time.sleep(30)

    md = cam.stop_recording()
    cam.close_camera()

    return md

def dual_rec():
    
    cam0 = RPiInsightCamera(preview=False, raw=False, framerate=framerate, cam_num=0)
    cam1 = RPiInsightCamera(preview=False, raw=False, framerate=framerate, cam_num=1)

    cam0.open_camera()
    cam1.open_camera()
    
    cam0.start_recording('/home/ant/catkin_ws/src/scoutRecord/AntobotDevices/AntobotCamera/antobot_devices_camera/test/temp')
    cam1.start_recording('/home/ant/catkin_ws/src/scoutRecord/AntobotDevices/AntobotCamera/antobot_devices_camera/test/temp')

    time.sleep(30)

    md0 = cam0.stop_recording()
    md1 = cam1.stop_recording()
    
    cam0.close_camera()
    cam1.close_camera()

    return (md0, md1)

def analyse_stamps(stamps):
    # print(stamps)

    pts = [x['SensorTimestamp'] for x in stamps]
    d = np.diff(pts)
    d_rel = d / (1000000000/framerate)
    
    (counts, edges) = np.histogram(d_rel)

    print(f"Target fps: {framerate}")
    print(f"Relative to {1/framerate} Hz")
    for bin in range(len(counts)):
        print(f"{edges[bin]:.5f} - {edges[bin+1]:.5f}: {counts[bin]}")



   


if __name__ == "__main__":

    framerate =30


    # md = rec()
    # analyse_stamps(md)

    # (md0, md1) = dual_rec()
    # analyse_stamps(md0)
    # analyse_stamps(md1)

    with open('/home/ant/catkin_ws/src/scoutRecord/AntoManager/antobot_manager_data/data/data/session_078/15_51_06.json', 'r') as file:
        txt = file.read()
        data = json.loads(txt)

    for md in data["cam_metadata"]:
        analyse_stamps(md)

    
