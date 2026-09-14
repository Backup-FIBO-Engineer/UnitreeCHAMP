import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from unitree_rl_deploy.config import DeployConfig
from unitree_rl_deploy.robot_files import available_robots, resolve_robot_files

PKG = Path(__file__).resolve().parents[1]


def _urdf_revolute(path: Path) -> set:
    root = ET.fromstring(path.read_text(encoding='utf-8'))
    names = set()
    for joint in root.findall('joint'):
        if joint.attrib.get('type') in ('revolute', 'continuous'):
            names.add(joint.attrib['name'])
    return names


def _mapped_joint_names(robot: str) -> list:
    data = yaml.safe_load((PKG / 'config' / f'{robot}_joints.yaml').read_text())
    params = data['/**']['ros__parameters']['joints_map']
    names = []
    for leg in ('left_front', 'right_front', 'left_hind', 'right_hind'):
        names.extend(params[leg][:3])
    return names


def test_go2_and_b2_yaml_match_urdf_and_45_obs():
    for robot in ('go2', 'b2'):
        cfg = DeployConfig.from_yaml(PKG / 'config' / f'{robot}_rl.yaml')
        urdf_joints = _urdf_revolute(PKG / 'urdf' / f'{robot}.urdf')
        mapped = set(_mapped_joint_names(robot))
        assert cfg.num_actions == 12
        assert cfg.num_obs == 45
        assert cfg.observation.single_size == 45
        for name in cfg.joint_names:
            assert name in urdf_joints, f'{robot}: {name} missing from URDF'
            assert name in mapped, f'{robot}: {name} missing from joints_map'
        assert set(cfg.joint_names) == mapped
        assert (PKG / 'mujoco' / f'{robot}.xml').is_file()
        assert (PKG / 'config' / f'{robot}_lowcmd.yaml').is_file()


def test_robot_files_are_policy_only():
    robots = available_robots(PKG)
    assert set(robots) == {'go2', 'b2'}
    for robot in robots:
        files = resolve_robot_files(PKG, robot)
        assert files.rl_yaml.is_file()
        assert files.urdf.is_file()
        assert files.mujoco_xml is not None
        assert files.lowcmd_yaml is not None
        assert not (PKG / 'config' / f'{robot}_gait.yaml').exists()
        assert not (PKG / 'config' / f'{robot}_body_pose.yaml').exists()


def test_launch_files_exist():
    launch = PKG / 'launch'
    assert (launch / 'mujoco_rl.launch.py').is_file()
    assert (launch / 'unitree_rl_sim2real.launch.py').is_file()
