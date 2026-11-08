from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    driver_package_dir = get_package_share_directory('robot_driver')
        
    config_path = os.path.join(driver_package_dir, 'config', 'driver_config.yaml')

    robot_node = Node(
            package='robot_driver',
            executable='robot_driver_node',
            output='screen',
            emulate_tty=True,
            #parameters=[config_path]
        )
    
    return LaunchDescription([
        robot_node
    ])