import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('champ_base_gait_planning')

    urdf_path = os.path.join(pkg_share, 'urdf', 'go2.urdf')
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)
    cyclone_xml = os.path.join(pkg_share, 'config', 'cyclonedds_ros_loopback.xml')

    gait_config = os.path.join(pkg_share, 'config', 'go2_gait.yaml')
    joints_config = os.path.join(pkg_share, 'config', 'go2_joints.yaml')
    links_config = os.path.join(pkg_share, 'config', 'go2_links.yaml')
    lowcmd_config = os.path.join(pkg_share, 'config', 'go2_lowcmd.yaml')

    common_params = [
        {'urdf': robot_description},
        gait_config,
        joints_config,
        links_config,
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'network_interface',
            default_value='',
            description='NIC cabled to the Go2 (e.g. eth0). Empty uses unitree_sdk2 default.',
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
        ),
        LogInfo(msg=[
            'Go2 Sim2Real: rt/lowcmd + rt/lowstate via unitree_sdk2. Sport must stay off.',
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
            executable='go2_dds_bridge',
            name='go2_dds_bridge',
            output='screen',
            parameters=[
                lowcmd_config,
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
            parameters=common_params + [
                {'orientation_from_imu': True},
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
