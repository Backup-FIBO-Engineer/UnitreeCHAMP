"""Policy runner + unitree_ros2_bridge on a real Unitree quadruped.

    ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=go2 \\
        network_interface:=eth0 policy:=/path/to/policy.pt
    ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=b2 \\
        network_interface:=eth0 policy:=/path/to/policy.pt
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from unitree_rl_deploy.robot_files import available_robots, load_ros_params, resolve_robot_files


def cyclonedds_uri(network_interface: str) -> str:
    return (
        '<CycloneDDS><Domain><General><Interfaces>'
        f'<NetworkInterface name="{network_interface}" priority="default" multicast="default"/>'
        '</Interfaces></General></Domain></CycloneDDS>'
    )


def launch_setup(context):
    pkg = Path(get_package_share_directory('unitree_rl_deploy'))
    files = resolve_robot_files(pkg, LaunchConfiguration('robot').perform(context))
    if files.lowcmd_yaml is None:
        raise FileNotFoundError(
            f"robot '{files.robot}' has no config/{files.robot}_lowcmd.yaml"
        )
    robot_description = ParameterValue(
        Command([files.urdf_command, ' ', str(files.urdf)]), value_type=str)
    rl_params = load_ros_params(files.rl_yaml)
    policy = LaunchConfiguration('policy').perform(context).strip()
    runner_params = [
        str(files.rl_yaml),
        {'imu_topic': LaunchConfiguration('imu_topic')},
    ]
    odom_topic = LaunchConfiguration('odom_topic').perform(context).strip()
    base_state_topic = LaunchConfiguration('base_state_topic').perform(context).strip()
    if odom_topic:
        runner_params.append({'odom_topic': odom_topic})
    if base_state_topic:
        runner_params.append({'base_state_topic': base_state_topic})
        if not odom_topic:
            # Do not keep yaml odom/ground_truth on a real robot.
            runner_params.append({'odom_topic': ''})
    if not odom_topic and not base_state_topic:
        runner_params.append({'odom_topic': '/odom'})
    lin_vel_frame = LaunchConfiguration('lin_vel_frame').perform(context).strip()
    if lin_vel_frame:
        runner_params.append({'lin_vel_frame': lin_vel_frame})
    if policy:
        runner_params.append({'policy': policy})

    bridge_params = [
        {'urdf': robot_description},
        str(files.joints_yaml),
        str(files.links_yaml),
        str(files.lowcmd_yaml),
        {'command_topic': rl_params.get('command_topic', 'joint_commands')},
    ]
    if rl_params.get('kp') is not None:
        bridge_params.append({'kp': float(rl_params['kp'])})
    if rl_params.get('kd') is not None:
        bridge_params.append({'kd': float(rl_params['kd'])})

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
            f'{files.robot} Sim2Real RL: policy_runner -> joint_commands -> /lowcmd. '
            'Sport must stay off. Do not mix with the Sport API.'
        ]),
        Node(
            package='unitree_rl_deploy',
            executable='policy_runner',
            name='policy_runner',
            output='screen',
            parameters=runner_params,
        ),
        Node(
            package='unitree_rl_deploy',
            executable='unitree_ros2_bridge',
            name='unitree_ros2_bridge',
            output='screen',
            parameters=bridge_params,
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
    pkg = Path(get_package_share_directory('unitree_rl_deploy'))
    robots = available_robots(pkg)
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            description='Robot name: ' + ', '.join(robots),
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='',
            description='NIC cabled to the robot (e.g. eth0). Empty keeps the shell environment.',
        ),
        DeclareLaunchArgument(
            'policy',
            default_value='',
            description='Actor file (.pt TorchScript/state_dict or .onnx). Empty holds default_angles.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='imu/data',
            description='sensor_msgs/Imu. Default is the bridge (/lowstate). '
                        'Real B2 external IMU: imu_topic:=/dog_imu_raw',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='',
            description='nav_msgs/Odometry for lin_vel. Empty uses /odom '
                        '(not yaml odom/ground_truth). Estimator example: odom_topic:=/odom',
        ),
        DeclareLaunchArgument(
            'base_state_topic',
            default_value='',
            description='Optional DLS dls2_interface/BaseState (world-frame linear vel). '
                        'Example: base_state_topic:=/base_state',
        ),
        DeclareLaunchArgument(
            'lin_vel_frame',
            default_value='',
            description='body or world for Odometry.twist. Empty uses yaml. '
                        '/base_state is always world.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
