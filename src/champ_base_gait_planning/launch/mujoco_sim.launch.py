"""CHAMP gait planning driving a MuJoCo simulation of any robot in this package.

    ros2 launch champ_base_gait_planning mujoco_sim.launch.py robot:=<robot> [headless:=true]

Needs mujoco/<robot>.xml (tools/generate_mjcf.py <robot> writes it from the URDF)
plus the CHAMP yaml files; simulator tuning comes from config/<robot>_sim.yaml.

With config/<robot>_body_pose.yaml present (body_pose_control:=auto, the default)
the IMU body roll/pitch loop runs between the user's /body_pose and CHAMP:
body_pose_controller_node reads /imu/data from the simulator and publishes
/body_pose/corrected, which quadruped_controller_node consumes as its body_pose.
floor_roll / floor_pitch (rad) tilt the simulated floor to exercise that loop.
"""

from ament_index_python.packages import get_package_share_directory
from champ_robot_files import available_robots, resolve_optional_feature, resolve_robot_files
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    files = resolve_robot_files(pkg_share, LaunchConfiguration('robot').perform(context))
    if files.mujoco_xml is None:
        raise FileNotFoundError(
            f"robot '{files.robot}' has no mujoco/{files.robot}.xml; run "
            f'tools/generate_mjcf.py {files.robot} (URDF primitives) or hand-write it')
    body_pose_control = resolve_optional_feature(
        files, LaunchConfiguration('body_pose_control').perform(context), 'body_pose_control',
        files.body_pose_yaml, f'config/{files.robot}_body_pose.yaml (IMU body roll/pitch loop)')
    robot_description = ParameterValue(
        Command([files.urdf_command, ' ', str(files.urdf)]), value_type=str)

    common_params = [
        {'urdf': robot_description},
        str(files.gait_yaml),
        str(files.joints_yaml),
        str(files.links_yaml),
    ]
    sim_params = common_params + ([str(files.sim_yaml)] if files.sim_yaml else [])

    controller_remappings = [
        ('joint_states', 'joint_commands'),
        ('foot_contacts', 'foot_contacts/planned'),
    ]
    actions = []
    if body_pose_control:
        # The user's /body_pose goes through the IMU loop first.
        controller_remappings.append(('body_pose', 'body_pose/corrected'))
        actions += [
            LogInfo(msg=[
                f'{files.robot}: IMU body roll/pitch loop on (config/{files.robot}_body_pose.yaml); '
                '/body_pose -> body_pose_controller_node -> /body_pose/corrected -> CHAMP',
            ]),
            Node(
                package='champ_base_gait_planning',
                executable='body_pose_controller_node',
                name='body_pose_controller_node',
                output='screen',
                parameters=common_params + [
                    str(files.body_pose_yaml),
                    {'imu_topic': LaunchConfiguration('imu_topic')},
                    {'desired_pose_topic': 'body_pose'},
                    {'command_pose_topic': 'body_pose/corrected'},
                ],
            ),
        ]
    else:
        actions.append(LogInfo(msg=[
            f'{files.robot}: IMU body roll/pitch loop off; /body_pose goes straight to CHAMP',
        ]))

    return actions + [
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
            remappings=controller_remappings,
        ),
        # MuJoCo consumes desired joints and publishes measured simulated state.
        Node(
            package='champ_base_gait_planning',
            executable='mujoco_sim.py',
            name='mujoco_sim',
            output='screen',
            parameters=sim_params + [
                {'xml_path': str(files.mujoco_xml)},
                {'headless': LaunchConfiguration('headless')},
                {'command_topic': 'joint_commands'},
                {'joint_state_topic': 'joint_states'},
                {'contact_topic': 'foot_contacts/sim'},
                {'odom_topic': 'odom/ground_truth'},
                {'imu_topic': LaunchConfiguration('imu_topic')},
                {'publish_rate': 50.0},
                {'realtime_factor': 1.0},
                {'command_timeout_sec': 0.5},
                {'floor_roll': ParameterValue(LaunchConfiguration('floor_roll'), value_type=float)},
                {'floor_pitch': ParameterValue(LaunchConfiguration('floor_pitch'), value_type=float)},
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
        # Estimator receives simulated measurements and measured contacts.
        Node(
            package='champ_base_gait_planning',
            executable='state_estimation_node',
            name='state_estimation_node',
            output='screen',
            parameters=common_params + [{'orientation_from_imu': False}],
            remappings=[('foot_contacts', 'foot_contacts/sim')],
            condition=IfCondition(LaunchConfiguration('start_estimator')),
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('champ_base_gait_planning')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            description='Robot name, one of: ' + ', '.join(available_robots(pkg_share)),
        ),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_estimator', default_value='true'),
        DeclareLaunchArgument('start_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument(
            'body_pose_control',
            default_value='auto',
            description='IMU body roll/pitch loop (body_pose_controller_node): auto = when '
                        'config/<robot>_body_pose.yaml exists, true, false',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='imu/data',
            description='sensor_msgs/Imu published by the simulator and read by the body pose loop',
        ),
        DeclareLaunchArgument(
            'floor_roll', default_value='0.0',
            description='Tilt of the simulated floor about +X (rad), to test the body pose loop',
        ),
        DeclareLaunchArgument(
            'floor_pitch', default_value='0.0',
            description='Tilt of the simulated floor about +Y (rad), to test the body pose loop',
        ),
        OpaqueFunction(function=launch_setup),
    ])
