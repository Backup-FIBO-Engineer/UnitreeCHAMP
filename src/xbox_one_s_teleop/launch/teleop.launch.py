"""Xbox One S (1708) Bluetooth teleop: joy_node + xbox_teleop_node.

    ros2 launch xbox_one_s_teleop teleop.launch.py robot:=go2
    ros2 launch xbox_one_s_teleop teleop.launch.py robot:=b2
    ros2 launch xbox_one_s_teleop teleop.launch.py robot:=xgo

robot:= loads gait + body_pose yaml from champ_base_gait_planning so stick
full-scale matches that robot's max vx/vy/wz and max roll/pitch. Mapping of
the pad itself is yaml: SDL axes 0/1/2/3, or Linux/xpadneo 0/1/3/4 when the
pad is listed as Xbox 360 (`driver:=auto`). No robot numbers in the pad yaml.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from xbox_one_s_teleop.sdl_devices import (
    missing_pad_error,
    pad_mapping_yaml,
    resolve_joy_device_id,
)


def _load_ros_params(path: Path) -> dict:
    data = yaml.safe_load(path.read_text()) or {}
    return data.get('/**', {}).get('ros__parameters', {}) or {}


def _robot_limits(robot: str) -> dict:
    if not robot:
        return {}
    try:
        champ = Path(get_package_share_directory('champ_base_gait_planning'))
    except PackageNotFoundError:
        return {}
    overlay = {}
    gait_path = champ / 'config' / f'{robot}_gait.yaml'
    if gait_path.is_file():
        gait = _load_ros_params(gait_path).get('gait', {})
        if 'max_linear_velocity_x' in gait:
            overlay['max_linear_x'] = float(gait['max_linear_velocity_x'])
        if 'max_linear_velocity_y' in gait:
            overlay['max_linear_y'] = float(gait['max_linear_velocity_y'])
        if 'max_angular_velocity_z' in gait:
            overlay['max_angular_z'] = float(gait['max_angular_velocity_z'])
    pose_path = champ / 'config' / f'{robot}_body_pose.yaml'
    if pose_path.is_file():
        body = _load_ros_params(pose_path).get('body_pose', {})
        if 'max_roll' in body:
            overlay['max_roll'] = float(body['max_roll'])
        if 'max_pitch' in body:
            overlay['max_pitch'] = float(body['max_pitch'])
        if 'desired_rate' in body:
            overlay['roll_rate'] = float(body['desired_rate'])
            overlay['pitch_rate'] = float(body['desired_rate'])
    return overlay


def launch_setup(context):
    pkg = get_package_share_directory('xbox_one_s_teleop')
    robot = LaunchConfiguration('robot').perform(context).strip()
    overlay = _robot_limits(robot)
    requested_id = LaunchConfiguration('device_id').perform(context).strip()
    device_name = LaunchConfiguration('device_name').perform(context).strip()
    device_id, sdl_devices = resolve_joy_device_id(requested_id)
    auto = requested_id.lower() in ('', 'auto')
    if auto and device_id is None:
        raise RuntimeError(missing_pad_error(sdl_devices))
    chosen_name = next((name for idx, name in sdl_devices if idx == device_id), '')
    mapping_name = pad_mapping_yaml(
        LaunchConfiguration('driver').perform(context),
        chosen_name,
    )
    mapping = str(Path(pkg) / 'config' / mapping_name)
    joy_params = {
        'device_id': device_id,
        'deadzone': 0.05,
        'autorepeat_rate': 20.0,
    }
    if device_name:
        joy_params['device_name'] = device_name
    actions = [
        LogInfo(msg=[f'Xbox teleop mapping {mapping_name}']),
        LogInfo(msg=[
            'SDL joysticks (not /dev/input/jsN): '
            + (', '.join(f'{idx}:{name}' for idx, name in sdl_devices) or 'none listed')
            + f'; joy_node device_id={device_id}'
            + (f' ({chosen_name})' if chosen_name else '')
            + (f', device_name={device_name}' if device_name else '')
        ]),
    ]
    if chosen_name and ('rustdesk' in chosen_name.lower() or 'uinput' in chosen_name.lower()):
        actions.append(LogInfo(msg=[
            'WARNING: joy_node would open a virtual pad (often RustDesk). '
            'List devices with: ros2 run joy joy_enumerate_devices '
            'then pass device_id:=<Xbox id> or device_name:="Xbox Wireless Controller"'
        ]))
    if overlay:
        actions.append(LogInfo(msg=[
            f'Xbox teleop limits from champ_base_gait_planning robot={robot}: {overlay}',
        ]))
    elif robot:
        actions.append(LogInfo(msg=[
            f"Xbox teleop: no gait/body_pose yaml for robot '{robot}', using pad yaml defaults",
        ]))
    actions += [
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_params],
        ),
        Node(
            package='xbox_one_s_teleop',
            executable='xbox_teleop_node',
            name='xbox_one_s_teleop',
            output='screen',
            parameters=[mapping, overlay],
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='go2',
            description='CHAMP robot stem (go2, b2, xgo): overlays gait and body_pose limits',
        ),
        DeclareLaunchArgument(
            'device_id',
            default_value='auto',
            description=(
                'SDL joystick index for joy_node. auto = first name containing Xbox '
                '(skips RustDesk/uinput). This is not /dev/input/jsN.'
            ),
        ),
        DeclareLaunchArgument(
            'device_name',
            default_value='',
            description=(
                'Exact SDL name for joy_node (overrides index). '
                'Example: device_name:="Xbox Wireless Controller"'
            ),
        ),
        DeclareLaunchArgument(
            'driver',
            default_value='auto',
            description=(
                'Axis map: auto = linux if SDL name is Xbox 360 (xpadneo), else sdl. '
                'linux = kernel js order (right stick 3/4). sdl = RX/RY on 2/3.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
