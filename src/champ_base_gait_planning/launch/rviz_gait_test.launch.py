"""CHAMP gait planning + state estimation + RViz for any robot in this package.

    ros2 launch champ_base_gait_planning rviz_gait_test.launch.py robot:=<robot>

The robot is selected purely by its files (urdf/<robot>.*, config/<robot>_*.yaml,
rviz/<robot>_gait.rviz); nothing robot-specific lives in this launch file.
"""

from ament_index_python.packages import get_package_share_directory
from champ_robot_files import available_robots, resolve_robot_files
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    files = resolve_robot_files(pkg_share, LaunchConfiguration('robot').perform(context))
    robot_description = ParameterValue(
        Command([files.urdf_command, ' ', str(files.urdf)]), value_type=str)
    use_sim_time = LaunchConfiguration('use_sim_time')

    common_params = [
        {'use_sim_time': use_sim_time},
        {'urdf': robot_description},
        str(files.gait_yaml),
        str(files.joints_yaml),
        str(files.links_yaml),
    ]

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}],
        ),
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
        ),
        Node(
            package='champ_base_gait_planning',
            executable='state_estimation_node',
            name='state_estimation_node',
            output='screen',
            parameters=common_params + [{'orientation_from_imu': False}],
        ),
    ]
    if files.rviz_config is not None:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(files.rviz_config)],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ))
    else:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ))
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            description='Robot name, one of: ' + ', '.join(available_robots(pkg_share)),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
