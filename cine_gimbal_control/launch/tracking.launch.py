from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('cine_gimbal_control')
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    params_file = LaunchConfiguration('params_file')
    gimbal_driver = LaunchConfiguration('gimbal_driver')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the parameters YAML file'
        ),
        DeclareLaunchArgument(
            'gimbal_driver',
            default_value='rs4',
            description='Gimbal driver selection: jc2804 or rs4'
        ),
        Node(
            package='cine_gimbal_control',
            executable='tracking_node',
            name='gimbal_tracking_node',
            output='screen',
            parameters=[params_file, {'gimbal_driver': gimbal_driver}]
        )
    ])
