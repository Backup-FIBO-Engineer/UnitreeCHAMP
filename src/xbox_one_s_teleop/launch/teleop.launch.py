"""Xbox One S (1708) Bluetooth teleop: joy_node + xbox_teleop_node.

    ros2 launch xbox_one_s_teleop teleop.launch.py robot:=go2
    ros2 launch xbox_one_s_teleop teleop.launch.py robot:=b2

robot:= loads max_cmd from unitree_rl_deploy config/<robot>_rl.yaml so stick
full-scale matches that policy's command clamp. Mapping of the pad itself is
yaml: SDL axes 0/1/2/3, or Linux/xpadneo 0/1/3/4 when the pad is listed as
Xbox 360 (`driver:=auto`). No robot numbers in the pad yaml.
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
        pkg = Path(get_package_share_directory('unitree_rl_deploy'))
    except PackageNotFoundError:
        return {}
    overlay = {}
    rl_path = pkg / 'config' / f'{robot}_rl.yaml'
    if rl_path.is_file():
        params = _load_ros_params(rl_path)
        max_cmd = params.get('max_cmd') or []
        if len(max_cmd) >= 3:
            overlay['max_linear_x'] = float(max_cmd[0])
            overlay['max_linear_y'] = float(max_cmd[1])
            overlay['max_angular_z'] = float(max_cmd[2])
    return overlay


def _overlay_bool(overlay, key, raw):
    value = (raw or '').strip().lower()
    if not value:
        return
    if value in ('true', '1', 'yes', 'on'):
        overlay[key] = True
    elif value in ('false', '0', 'no', 'off'):
        overlay[key] = False
    else:
        raise RuntimeError(f'{key}:= must be true or false, got {raw!r}')


def launch_setup(context):
    pkg = get_package_share_directory('xbox_one_s_teleop')
    robot = LaunchConfiguration('robot').perform(context).strip()
    overlay = _robot_limits(robot)
    _overlay_bool(overlay, 'invert_vx', LaunchConfiguration('invert_vx').perform(context))
    _overlay_bool(overlay, 'invert_vy', LaunchConfiguration('invert_vy').perform(context))
    _overlay_bool(overlay, 'invert_yaw', LaunchConfiguration('invert_yaw').perform(context))
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
            f'Xbox teleop limits/signs overlay: {overlay}',
        ]))
    elif robot:
        actions.append(LogInfo(msg=[
            f"Xbox teleop: no {robot}_rl.yaml max_cmd overlay, using pad yaml defaults",
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
            description='Robot stem (go2, b2): overlays max_cmd from unitree_rl_deploy',
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
        DeclareLaunchArgument(
            'invert_vx',
            default_value='',
            description=(
                'Overlay stick-forward sign. Empty keeps the mapping yaml. '
                'true/false if /cmd_vel linear.x is the wrong sign (robot only walks backward).'
            ),
        ),
        DeclareLaunchArgument(
            'invert_vy',
            default_value='',
            description='Overlay left-stick X sign. Empty keeps the mapping yaml.',
        ),
        DeclareLaunchArgument(
            'invert_yaw',
            default_value='',
            description='Overlay right-stick X yaw sign. Empty keeps the mapping yaml.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
