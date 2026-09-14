"""Policy runner + MuJoCo.

    ros2 launch unitree_rl_deploy mujoco_rl.launch.py robot:=go2
    ros2 launch unitree_rl_deploy mujoco_rl.launch.py robot:=b2 policy:=/path/to/policy.pt
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from unitree_rl_deploy.robot_files import available_robots, load_ros_params, resolve_robot_files


def launch_setup(context):
    pkg = Path(get_package_share_directory('unitree_rl_deploy'))
    files = resolve_robot_files(pkg, LaunchConfiguration('robot').perform(context))
    if files.mujoco_xml is None:
        raise FileNotFoundError(f"robot '{files.robot}' has no mujoco/{files.robot}.xml")
    robot_description = ParameterValue(
        Command([files.urdf_command, ' ', str(files.urdf)]), value_type=str)
    rl_params = load_ros_params(files.rl_yaml)
    policy = LaunchConfiguration('policy').perform(context).strip()
    runner_params = [
        str(files.rl_yaml),
        {'imu_topic': LaunchConfiguration('imu_topic')},
    ]
    if policy:
        runner_params.append({'policy': policy})

    sim_params = [
        {'urdf': robot_description},
        str(files.joints_yaml),
        str(files.links_yaml),
    ]
    if files.sim_yaml:
        sim_params.append(str(files.sim_yaml))

    return [
        LogInfo(msg=[
            f'{files.robot} MuJoCo RL: policy_runner -> joint_commands -> mujoco_sim. '
            '/cmd_vel steers the policy.'
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
            executable='mujoco_sim.py',
            name='mujoco_sim',
            output='screen',
            parameters=sim_params + [
                {'xml_path': str(files.mujoco_xml)},
                {'headless': LaunchConfiguration('headless')},
                {'command_topic': rl_params.get('command_topic', 'joint_commands')},
                {'joint_state_topic': rl_params.get('joint_state_topic', 'joint_states')},
                {'odom_topic': 'odom/ground_truth'},
                {'imu_topic': LaunchConfiguration('imu_topic')},
                {'publish_rate': 50.0},
                {'realtime_factor': 1.0},
                {'command_timeout_sec': 0.5},
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(LaunchConfiguration('start_robot_state_publisher')),
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
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument(
            'policy',
            default_value='',
            description='Actor file (.pt TorchScript/state_dict or .onnx). Empty holds default_angles.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='imu/data',
            description='sensor_msgs/Imu from mujoco_sim',
        ),
        OpaqueFunction(function=launch_setup),
    ])
