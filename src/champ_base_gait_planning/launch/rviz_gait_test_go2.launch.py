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

    gait_config = os.path.join(pkg_share, 'config', 'go2_gait.yaml')
    joints_config = os.path.join(pkg_share, 'config', 'go2_joints.yaml')
    links_config = os.path.join(pkg_share, 'config', 'go2_links.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'go2_gait.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    common_params = [
        {'use_sim_time': use_sim_time},
        {'urdf': robot_description},
        gait_config,
        joints_config,
        links_config,
    ]

    controller_params = common_params + [
        {'publish_joint_states': True},
        {'publish_foot_contacts': True},
        {'publish_joint_control': False},
        {'gazebo': False},
        {'loop_rate': 200.0},
    ]

    estimator_params = common_params + [
        {'orientation_from_imu': False},
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='champ_base_gait_planning',
            executable='quadruped_controller_node',
            name='quadruped_controller_node',
            output='screen',
            parameters=controller_params,
        ),
        Node(
            package='champ_base_gait_planning',
            executable='state_estimation_node',
            name='state_estimation_node',
            output='screen',
            parameters=estimator_params,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
