import argparse
from picamera2 import Picamera2, Preview

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, choices=[0,1], default=0, help="camera number")
    parser.add_argument("--preview", help="show frame in preview window")
    args = parser.parse_args()

    cam = Picamera2(camera_num=args.camera)
    
    config = cam.create_video_configuration(
            # Put camera sensor into mode 1 (i.e cam.sensor_modes[1]).
            # The best way is to specify output_size and bit_depth 
            # (Picamera2 docs, p.23)
            sensor={
                'output_size': (2028,1080),
                'bit_depth': 12
            }, 
            # set frame rate; 50 fps max in this sensor mode
            controls={
                'FrameRate': 30
            },
            main={
                'size': (2028,1080),
                'format': 'RGB888'
            },
        )
    cam.configure(config)

    if args.preview:
        cam.start_preview(Preview.QTGL)
    
    cam.start()

    print(f"Focus assist tool. Adjust lens on camera {args.camera}.")
    print("Higher FocusFoM is better.")

    while True:
        md = cam.capture_metadata()
        focusfom = md["FocusFoM"]
        print(f"FocusFoM: {focusfom}", end='\r')


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('Stopping')

