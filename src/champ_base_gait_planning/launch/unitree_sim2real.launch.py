"""CHAMP -> unitree_ros2_bridge -> /lowcmd (unitree_ros2) on a real Unitree quadruped.

    ros2 launch champ_base_gait_planning unitree_sim2real.launch.py robot:=<robot> \
        network_interface:=eth0

Robot selection is file based: urdf/<robot>.*, config/<robot>_{gait,joints,links}.yaml
for CHAMP and config/<robot>_lowcmd.yaml for the bridge gains/motor mode. The joint
names, joint limits and base/IMU frames the bridge uses all come from those files.

The robot is a participant of the same ROS 2 graph (unitree_ros2: CycloneDDS on
domain 0 over the cabled NIC). network_interface:=<nic> sets RMW_IMPLEMENTATION and
CYCLONEDDS_URI for every node of this launch, exactly like unitree_ros2/setup.sh;
leave it empty when that setup.sh is already sourced. ROS_DOMAIN_ID is never set
here: the robot lives on domain 0, so it must be 0 (or unset) in the shell.
Sport/motion-control services must be off on the robot (the bridge calls ReleaseMode).
"""

from ament_index_python.packages import get_package_share_directory
from champ_robot_files import available_robots, resolve_robot_files
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def cyclonedds_uri(network_interface):
    """Inline CycloneDDS config pinning the NIC, as in unitree_ros2/setup.sh."""
    return (
        '<CycloneDDS><Domain><General><Interfaces>'
        f'<NetworkInterface name="{network_interface}" priority="default" multicast="default"/>'
        '</Interfaces></General></Domain></CycloneDDS>'
    )


def launch_setup(context):
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    files = resolve_robot_files(pkg_share, LaunchConfiguration('robot').perform(context))
    if files.lowcmd_yaml is None:
        raise FileNotFoundError(
            f"robot '{files.robot}' has no config/{files.robot}_lowcmd.yaml (unitree_ros2_bridge "
            'gains, motor_mode, contact threshold); it cannot be driven over /lowcmd')
    robot_description = ParameterValue(
        Command([files.urdf_command, ' ', str(files.urdf)]), value_type=str)

    common_params = [
        {'urdf': robot_description},
        str(files.gait_yaml),
        str(files.joints_yaml),
        str(files.links_yaml),
    ]

    actions = []
    network_interface = LaunchConfiguration('network_interface').perform(context).strip()
    if network_interface:
        actions += [
            SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
            SetEnvironmentVariable(name='CYCLONEDDS_URI', value=cyclonedds_uri(network_interface)),
            LogInfo(msg=[f'CycloneDDS bound to {network_interface} for every node of this launch']),
        ]
    else:
        actions.append(LogInfo(msg=[
            'network_interface is empty: using RMW_IMPLEMENTATION / CYCLONEDDS_URI from the shell '
            '(source unitree_ros2/setup.sh)',
        ]))

    return actions + [
        LogInfo(msg=[
            f'{files.robot} Sim2Real: /lowcmd + /lowstate via unitree_ros2. Sport must stay off.',
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
            executable='unitree_ros2_bridge',
            name='unitree_ros2_bridge',
            output='screen',
            parameters=common_params + [
                str(files.lowcmd_yaml),
                {'command_topic': LaunchConfiguration('command_topic')},
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            description='Robot name, one of: ' + ', '.join(available_robots(pkg_share)),
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='',
            description='NIC cabled to the robot (e.g. eth0): sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp '
                        'and CYCLONEDDS_URI for this launch. Empty keeps the shell environment '
                        '(unitree_ros2/setup.sh).',
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='joint_commands',
            description='CHAMP planned joints consumed by unitree_ros2_bridge',
        ),
        OpaqueFunction(function=launch_setup),
    ])
