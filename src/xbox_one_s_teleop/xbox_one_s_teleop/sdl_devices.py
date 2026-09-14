"""SDL joystick index is not /dev/input/jsN.

joy_node (SDL2) enumerates every joystick SDL can see. A virtual pad such as
RustDesk UInput Keyboard is often index 0 even when the only Linux js node is
the Xbox at /dev/input/js0.
"""
from __future__ import annotations

import re
import subprocess
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

Device = Tuple[int, str]

_ENUM_LINE = re.compile(
    r'^\s*(\d+)\s*:\s*\S+\s*:\s*\S+\s*:\s*\S+\s*:\s*(.+?)\s*$'
)


def parse_joy_enumerate(text: str) -> List[Device]:
    devices: List[Device] = []
    for line in text.splitlines():
        match = _ENUM_LINE.match(line)
        if not match:
            continue
        devices.append((int(match.group(1)), match.group(2).strip()))
    return devices


def _name_looks_like_xbox(name: str) -> bool:
    lower = name.lower()
    return 'xbox' in lower or 'x-box' in lower


def _name_looks_like_virtual(name: str) -> bool:
    lower = name.lower()
    return 'rustdesk' in lower or 'uinput' in lower


def choose_joystick_id(
    devices: Sequence[Device],
    requested: str = 'auto',
) -> Optional[int]:
    """Return an SDL device_id, or None if nothing usable is listed."""
    raw = (requested or 'auto').strip()
    if raw.lower() not in ('', 'auto'):
        return int(raw)
    if not devices:
        return None
    for index, name in devices:
        if _name_looks_like_xbox(name) and not _name_looks_like_virtual(name):
            return index
    for index, name in devices:
        if not _name_looks_like_virtual(name):
            return index
    return None


def missing_pad_error(devices: Sequence[Device]) -> str:
    listed = ', '.join(f'{idx}:{name}' for idx, name in devices) or 'none'
    return (
        'No Xbox pad in the SDL joystick list (joy_node cannot see it). '
        f'Currently listed: {listed}. '
        '/dev/input/js0 is not the Xbox (often mouce-library-fake-mouse / RustDesk). '
        'The 1708 must appear in /proc/bus/input/devices with a jsN handler. '
        'If dmesg shows microsoft 0005:045E:02FD parse failed / error -22, '
        'install xpadneo and reboot (hid-microsoft cannot parse 1708 Bluetooth). '
    )


def list_sdl_joysticks(timeout_sec: float = 5.0) -> List[Device]:
    try:
        exe = Path(get_package_prefix('joy')) / 'lib' / 'joy' / 'joy_enumerate_devices'
    except PackageNotFoundError:
        return []
    if not exe.is_file():
        return []
    try:
        proc = subprocess.run(
            [str(exe)],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    return parse_joy_enumerate(proc.stdout or '')


def resolve_joy_device_id(requested: str = 'auto') -> Tuple[Optional[int], List[Device]]:
    devices = list_sdl_joysticks()
    return choose_joystick_id(devices, requested), devices
