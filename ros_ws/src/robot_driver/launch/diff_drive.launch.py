from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robot_node = Node(
            package='robot_driver',
            executable='robot_driver_node',
            output='screen',
            emulate_tty=True
        )
    
    return LaunchDescription([
        robot_node
    ])