#!/usr/bin/env python3
"""Lock config/<robot>_gait.yaml to the numbers used by tools/verify_champ_<robot>.cpp.

    verify_gait_yaml.py [go2|b2]
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

EXPECTED = {
    'go2': {
        'knee_orientation': '">>"',
        'odom_scaler': 0.9,
        'max_linear_velocity_x': 0.50,
        'max_linear_velocity_y': 0.15,
        'max_angular_velocity_z': 0.6,
        'com_x_translation': 0.0,
        'swing_height': 0.08,
        'stance_depth': 0.0,
        'stance_duration': 0.25,
        'nominal_height': 0.30,
    },
    'b2': {
        'knee_orientation': '">>"',
        'odom_scaler': 0.9,
        'max_linear_velocity_x': 0.60,
        'max_linear_velocity_y': 0.20,
        'max_angular_velocity_z': 0.6,
        'com_x_translation': 0.0,
        'swing_height': 0.10,
        'stance_depth': 0.0,
        'stance_duration': 0.30,
        'nominal_height': 0.50,
    },
}


def main() -> int:
    robot = sys.argv[1] if len(sys.argv) > 1 else 'go2'
    if robot not in EXPECTED:
        print(f'unknown robot {robot!r}; choose from {sorted(EXPECTED)}')
        return 2
    path = Path(__file__).resolve().parents[1] / 'config' / f'{robot}_gait.yaml'
    text = path.read_text()
    failures = 0
    for key, expected in EXPECTED[robot].items():
        match = re.search(rf'{key}:\s*(\S+)', text)
        if match is None:
            print(f'[FAIL] {key} missing in {path}')
            failures += 1
            continue
        got = match.group(1)
        if isinstance(expected, str):
            ok = got == expected
        else:
            ok = abs(float(got) - float(expected)) < 1e-9
        print(f'[{"PASS" if ok else "FAIL"}] {key}: yaml={got} lock={expected}')
        failures += 0 if ok else 1
    print(f'\nResult: {failures} failed checks')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
