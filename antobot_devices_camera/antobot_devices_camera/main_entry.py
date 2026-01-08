#!/usr/bin/env python3
import sys
import rclpy
from antobot_devices_camera.scouting_supervisor import ScoutingSupervisor

def main(args=None):
    rclpy.init(args=args)

    config_path = '/home/scouting/ros2_ws/src/acCamera/antobot_devices_camera/config/scouting_config.yaml'
    
    node = None
    try:
        node = ScoutingSupervisor(config_path)
        
        # Use MultiThreadedExecutor so Service Callbacks (Record Start/Stop)
        # do not block the Transfer Manager callbacks or other ROS events.
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=6)
        executor.add_node(node)
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Critical System Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()