from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('cine_gimbal_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='cine_gimbal_control',
            executable='tracking_node',
            name='gimbal_tracking_node',
            output='screen',
            parameters=[params_file]
        )
    ])
