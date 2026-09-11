"""CHAMP -> unitree_dds_bridge -> rt/lowcmd on a real Unitree quadruped.

    ros2 launch champ_base_gait_planning unitree_sim2real.launch.py robot:=<robot> \
        network_interface:=eth0

Robot selection is file based: urdf/<robot>.*, config/<robot>_{gait,joints,links}.yaml
for CHAMP and config/<robot>_lowcmd.yaml for the bridge gains/motor mode. The joint
names, joint limits and base/IMU frames the bridge uses all come from those files.
Sport/motion-control services must be off on the robot (the bridge calls ReleaseMode).
"""

import os

from ament_index_python.packages import get_package_share_directory
from champ_robot_files import available_robots, resolve_robot_files
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    files = resolve_robot_files(pkg_share, LaunchConfiguration('robot').perform(context))
    if files.lowcmd_yaml is None:
        raise FileNotFoundError(
            f"robot '{files.robot}' has no config/{files.robot}_lowcmd.yaml (unitree_dds_bridge "
            'gains, motor_mode, contact threshold); it cannot be driven over rt/lowcmd')
    robot_description = ParameterValue(
        Command([files.urdf_command, ' ', str(files.urdf)]), value_type=str)

    common_params = [
        {'urdf': robot_description},
        str(files.gait_yaml),
        str(files.joints_yaml),
        str(files.links_yaml),
    ]

    return [
        LogInfo(msg=[
            f'{files.robot} Sim2Real: rt/lowcmd + rt/lowstate via unitree_sdk2. Sport must stay off.',
        ]),
        Node(
            package='champ_base_gait_planning',
            executable='quadruped_controller_node',
            name='quadruped_controller_node',
            output='screen',
            parameters=common_params + [
                {'publish_joint_states': True},
                {'publish_foot_contacts': True},
                {'publish_joint_control': False},
                {'gazebo': False},
                {'loop_rate': 200.0},
            ],
            remappings=[
                ('joint_states', LaunchConfiguration('command_topic')),
                ('foot_contacts', 'foot_contacts/planned'),
            ],
        ),
        Node(
            package='champ_base_gait_planning',
            executable='unitree_dds_bridge',
            name='unitree_dds_bridge',
            output='screen',
            parameters=common_params + [
                str(files.lowcmd_yaml),
                {
                    'command_topic': LaunchConfiguration('command_topic'),
                    'network_interface': LaunchConfiguration('network_interface'),
                },
            ],
        ),
        Node(
            package='champ_base_gait_planning',
            executable='state_estimation_node',
            name='state_estimation_node',
            output='screen',
            parameters=common_params + [{'orientation_from_imu': True}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    cyclone_xml = os.path.join(pkg_share, 'config', 'cyclonedds_ros_loopback.xml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            description='Robot name, one of: ' + ', '.join(available_robots(pkg_share)),
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='',
            description='NIC cabled to the robot (e.g. eth0). Empty uses the unitree_sdk2 default.',
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='joint_commands',
            description='CHAMP planned joints consumed by the DDS bridge',
        ),
        DeclareLaunchArgument(
            'ros_loopback_dds',
            default_value='true',
            description='Bind ROS 2 CycloneDDS to lo so it does not fight unitree_sdk2 on eth',
        ),
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value='file://' + cyclone_xml,
            condition=IfCondition(LaunchConfiguration('ros_loopback_dds')),
        ),
        OpaqueFunction(function=launch_setup),
    ])
