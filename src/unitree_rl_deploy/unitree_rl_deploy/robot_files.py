"""Robot files for policy deploy.

A robot is the stem of urdf/<robot>.urdf plus matching yaml:

    urdf/<robot>.urdf
    config/<robot>_rl.yaml          observation / default pose / action scale
    config/<robot>_joints.yaml      joints_map (leg hip/thigh/calf names)
    config/<robot>_links.yaml       links_map (base, imu, leg links)
    config/<robot>_sim.yaml         optional MuJoCo tuning
    config/<robot>_lowcmd.yaml      optional unitree_ros2_bridge gains
    mujoco/<robot>.xml              optional MuJoCo model
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

LEGS = ('left_front', 'right_front', 'left_hind', 'right_hind')
JOINTS_PER_LEG = 3
PACKAGE_NAME = 'unitree_rl_deploy'
SOURCE_PACKAGE_DIR = Path(__file__).resolve().parents[1]


@dataclass
class RobotFiles:
    robot: str
    package_dir: Path
    urdf: Path
    rl_yaml: Path
    joints_yaml: Path
    links_yaml: Path
    sim_yaml: Optional[Path]
    lowcmd_yaml: Optional[Path]
    mujoco_xml: Optional[Path]

    @property
    def urdf_command(self) -> str:
        return 'cat'


def available_robots(package_dir: Path | str = SOURCE_PACKAGE_DIR) -> List[str]:
    config_dir = Path(package_dir) / 'config'
    names = []
    for path in sorted(config_dir.glob('*_rl.yaml')):
        robot = path.name[:-8]
        if (Path(package_dir) / 'urdf' / f'{robot}.urdf').is_file():
            names.append(robot)
    return names


def resolve_robot_files(package_dir: Path | str, robot: str) -> RobotFiles:
    package_dir = Path(package_dir)
    robot = robot.strip()
    robots = available_robots(package_dir)
    if not robot or robot not in robots:
        raise FileNotFoundError(
            f"unknown robot '{robot}'. Available (urdf + config/<name>_rl.yaml): "
            f"{', '.join(robots) or 'none'}"
        )

    def required(rel: str) -> Path:
        path = package_dir / rel
        if not path.exists():
            raise FileNotFoundError(f"robot '{robot}' is missing {rel}")
        return path

    def optional(rel: str) -> Optional[Path]:
        path = package_dir / rel
        return path if path.exists() else None

    return RobotFiles(
        robot=robot,
        package_dir=package_dir,
        urdf=required(f'urdf/{robot}.urdf'),
        rl_yaml=required(f'config/{robot}_rl.yaml'),
        joints_yaml=required(f'config/{robot}_joints.yaml'),
        links_yaml=required(f'config/{robot}_links.yaml'),
        sim_yaml=optional(f'config/{robot}_sim.yaml'),
        lowcmd_yaml=optional(f'config/{robot}_lowcmd.yaml'),
        mujoco_xml=optional(f'mujoco/{robot}.xml'),
    )


def load_ros_params(*yaml_paths: Optional[Path | str]) -> Dict[str, Any]:
    import yaml

    merged: Dict[str, Any] = {}

    def merge(dst: dict, src: dict) -> None:
        for key, value in src.items():
            if isinstance(value, dict) and isinstance(dst.get(key), dict):
                merge(dst[key], value)
            else:
                dst[key] = value

    for path in yaml_paths:
        if path is None:
            continue
        document = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
        found = False
        for node_value in document.values():
            if isinstance(node_value, dict) and 'ros__parameters' in node_value:
                merge(merged, node_value['ros__parameters'] or {})
                found = True
        if not found:
            raise ValueError(f'{path}: no ros__parameters block found')
    return merged
