from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    line_follower_package_dir = get_package_share_directory('line_follower_core')
    
    config_path = os.path.join(line_follower_package_dir, 'config', 'lf_config.yaml')


    line_follower_node = Node(
            package='line_follower_core',
            executable='lf_node',
            #parameters=[config_path]
        )
    #при считывании параметров в rqt появляется два одинаковых топика один с ошибкой и дебаг не работает
    return LaunchDescription([
        line_follower_node
    ])