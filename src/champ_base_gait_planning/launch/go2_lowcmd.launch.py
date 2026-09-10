import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    return LaunchDescription([
        DeclareLaunchArgument('network_interface', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'go2_sim2real.launch.py')
            ),
            launch_arguments={
                'network_interface': LaunchConfiguration('network_interface'),
            }.items(),
        ),
    ])
