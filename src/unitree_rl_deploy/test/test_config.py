import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from unitree_rl_deploy.config import DeployConfig

ROOT = Path(__file__).resolve().parents[3]
CHAMP = ROOT / 'src' / 'champ_base_gait_planning'
RL = Path(__file__).resolve().parents[1]


def _urdf_revolute(path: Path) -> set:
    root = ET.fromstring(path.read_text(encoding='utf-8'))
    names = set()
    for joint in root.findall('joint'):
        if joint.attrib.get('type') in ('revolute', 'continuous'):
            names.add(joint.attrib['name'])
    return names


def _champ_joint_names(robot: str) -> list:
    data = yaml.safe_load((CHAMP / 'config' / f'{robot}_joints.yaml').read_text())
    params = data['/**']['ros__parameters']['joints_map']
    names = []
    for leg in ('left_front', 'right_front', 'left_hind', 'right_hind'):
        names.extend(params[leg][:3])
    return names


def test_go2_and_b2_yaml_match_urdf_and_45_obs():
    for robot in ('go2', 'b2'):
        cfg = DeployConfig.from_yaml(RL / 'config' / f'{robot}_rl.yaml')
        urdf_joints = _urdf_revolute(CHAMP / 'urdf' / f'{robot}.urdf')
        champ = set(_champ_joint_names(robot))
        assert cfg.num_actions == 12
        assert cfg.num_obs == 45
        assert cfg.observation.single_size == 45
        for name in cfg.joint_names:
            assert name in urdf_joints, f'{robot}: {name} missing from URDF'
            assert name in champ, f'{robot}: {name} missing from joints_map'
        assert set(cfg.joint_names) == champ
        assert (CHAMP / 'mujoco' / f'{robot}.xml').is_file()
        assert (CHAMP / 'config' / f'{robot}_lowcmd.yaml').is_file()


def test_launch_files_exist():
    launch = RL / 'launch'
    assert (launch / 'mujoco_rl.launch.py').is_file()
    assert (launch / 'unitree_rl_sim2real.launch.py').is_file()
