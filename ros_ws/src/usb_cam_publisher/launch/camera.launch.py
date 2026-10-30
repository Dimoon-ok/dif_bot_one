from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    camera_package_dir = get_package_share_directory('usb_cam_publisher')
    
    config_path = os.path.join(camera_package_dir, 'config', 'camera_config.yaml')


    camera_node = Node(
            package='usb_cam_publisher',
            executable='camera_node',
            parameters=[config_path]
        )
    
    return LaunchDescription([
        camera_node
    ])