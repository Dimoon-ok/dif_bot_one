import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    driver_share = get_package_share_directory('robot_driver')
    camera_share = get_package_share_directory('usb_cam_publisher')

    driver_launch_path = os.path.join(driver_share, 'launch', 'diff_drive.launch.py')
    camera_launch_path = os.path.join(camera_share, 'launch', 'camera.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path)
        )
    ])
