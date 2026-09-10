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

    urdf_path = os.path.join(pkg_share, 'urdf', 'go2.urdf')
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)
    xml_path = os.path.join(pkg_share, 'mujoco', 'go2.xml')

    gait_config = os.path.join(pkg_share, 'config', 'go2_gait.yaml')
    joints_config = os.path.join(pkg_share, 'config', 'go2_joints.yaml')
    links_config = os.path.join(pkg_share, 'config', 'go2_links.yaml')

    headless = LaunchConfiguration('headless')
    start_estimator = LaunchConfiguration('start_estimator')
    start_robot_state_publisher = LaunchConfiguration('start_robot_state_publisher')

    common_params = [
        {'urdf': robot_description},
        gait_config,
        joints_config,
        links_config,
    ]

    go2_joints = [
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_estimator', default_value='true'),
        DeclareLaunchArgument('start_robot_state_publisher', default_value='true'),

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

        Node(
            package='champ_base_gait_planning',
            executable='mujoco_sim.py',
            name='mujoco_sim',
            output='screen',
            parameters=[
                {'headless': headless},
                {'xml_path': xml_path},
                {'joint_names': go2_joints},
                {'foot_geom_names': ['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot']},
                {'imu_body_name': 'imu'},
                {'base_frame': 'base'},
                {'command_topic': 'joint_commands'},
                {'joint_state_topic': 'joint_states'},
                {'contact_topic': 'foot_contacts/sim'},
                {'odom_topic': 'odom/ground_truth'},
                {'publish_rate': 50.0},
                {'realtime_factor': 1.0},
                {'command_timeout_sec': 0.5},
                # Calf URDF velocity limit is 15.70 rad/s; hip/thigh allow 30.1.
                {'max_joint_velocity': 15.70},
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
