import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('champ_base_gait_planning')

    urdf_path = os.path.join(pkg_share, 'urdf', 'xgo_rviz.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    gait_config = os.path.join(pkg_share, 'config', 'xgo_rviz_gait.yaml')
    joints_config = os.path.join(pkg_share, 'config', 'xgo_joints.yaml')
    links_config = os.path.join(pkg_share, 'config', 'xgo_links.yaml')

    headless = LaunchConfiguration('headless')
    start_estimator = LaunchConfiguration('start_estimator')
    start_robot_state_publisher = LaunchConfiguration('start_robot_state_publisher')

    common_params = [
        {'urdf': robot_description},
        gait_config,
        joints_config,
        links_config,
    ]

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_estimator', default_value='true'),
        DeclareLaunchArgument('start_robot_state_publisher', default_value='true'),

        # CHAMP desired joint positions are commands, not measured joint states.
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
                ('joint_states', 'joint_commands'),
                ('foot_contacts', 'foot_contacts/planned'),
            ],
        ),

        # MuJoCo consumes desired joints and publishes measured simulated state.
        Node(
            package='champ_base_gait_planning',
            executable='mujoco_sim.py',
            name='mujoco_sim',
            output='screen',
            parameters=[
                {'headless': headless},
                {'command_topic': 'joint_commands'},
                {'joint_state_topic': 'joint_states'},
                {'contact_topic': 'foot_contacts/sim'},
                {'odom_topic': 'odom/ground_truth'},
                {'publish_rate': 50.0},
                {'realtime_factor': 1.0},
                {'command_timeout_sec': 0.5},
                # URDF velocity="1.5" is a SolidWorks exporter placeholder; the real XGO
                # bus servo runs 0.1 s/60deg = 10.47 rad/s. The CHAMP trot needs up to
                # ~8.7 rad/s, so 1.5 would cut walking speed to ~57% (verified in physics).
                {'max_joint_velocity': 10.47},
            ],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(start_robot_state_publisher),
        ),

        # Estimator now receives simulated measurements and measured contacts.
        Node(
            package='champ_base_gait_planning',
            executable='state_estimation_node',
            name='state_estimation_node',
            output='screen',
            parameters=common_params + [{'orientation_from_imu': False}],
            remappings=[('foot_contacts', 'foot_contacts/sim')],
            condition=IfCondition(start_estimator),
        ),
    ])
