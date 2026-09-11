"""Short alias of unitree_sim2real.launch.py.

    ros2 launch champ_base_gait_planning unitree_lowcmd.launch.py robot:=<robot> network_interface:=eth0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    return LaunchDescription([
        DeclareLaunchArgument('robot', description='Robot name (urdf/<robot>.*)'),
        DeclareLaunchArgument('network_interface', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'unitree_sim2real.launch.py')
            ),
            launch_arguments={
                'robot': LaunchConfiguration('robot'),
                'network_interface': LaunchConfiguration('network_interface'),
            }.items(),
        ),
    ])
