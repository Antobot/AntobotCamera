from setuptools import setup
from glob import glob
import os

package_name = 'antobot_devices_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),                 
        ('share/' + package_name, ['package.xml']),     
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jinhuan Liu',
    maintainer_email='jinhuan.liu@antobot.ai',
    description='Antobot device camera nodes and services',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_record_manager = antobot_devices_camera.camera_record_manager:main',
            'transfer_manager = antobot_devices_camera.transfer_manager:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': '/usr/bin/env python3'
        }
    }
)
