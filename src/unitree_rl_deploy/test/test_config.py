import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from unitree_rl_deploy.config import DeployConfig, parse_ros_bool
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


def test_go2_and_b2_yaml_match_urdf_and_260_obs():
    for robot in ('go2', 'b2'):
        cfg = DeployConfig.from_yaml(PKG / 'config' / f'{robot}_rl.yaml')
        urdf_joints = _urdf_revolute(PKG / 'urdf' / f'{robot}.urdf')
        mapped = set(_mapped_joint_names(robot))
        assert cfg.num_actions == 12
        assert cfg.num_obs == 260
        assert cfg.observation.single_size == 52
        assert cfg.observation.history == 5
        assert cfg.observation.clock_size == 4
        assert cfg.observation.terms[0] == 'lin_vel'
        assert cfg.observation.terms[-1] == 'clock'
        assert cfg.observation.needs_lin_vel is True
        assert cfg.observation.needs_clock is True
        assert cfg.odom_topic == 'odom/ground_truth'
        assert cfg.lin_vel_frame == 'body'
        assert cfg.use_filter_actions is True
        assert cfg.filter_alpha == pytest.approx(0.8)
        assert cfg.action_scale == pytest.approx(0.5)
        assert cfg.clip_actions == pytest.approx(3.0)
        np.testing.assert_allclose(cfg.max_cmd, [0.8, 0.25, 0.5])
        np.testing.assert_allclose(cfg.observation.lin_vel_scale, 1.0)
        np.testing.assert_allclose(cfg.observation.ang_vel_scale, 1.0)
        np.testing.assert_allclose(cfg.observation.dof_vel_scale, 1.0)
        np.testing.assert_allclose(cfg.observation.cmd_scale, [1.0, 1.0, 1.0])
        assert cfg.joint_names == [
            'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
            'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
            'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint',
        ]
        np.testing.assert_allclose(
            cfg.default_angles,
            [0.0, 0.0, 0.0, 0.0, 0.9, 0.9, 0.9, 0.9, -1.8, -1.8, -1.8, -1.8],
        )
        assert list(cfg.hip_indices) == [0, 1, 2, 3]
        assert cfg.lin_vel_com_offset.shape == (3,)
        assert float(np.linalg.norm(cfg.lin_vel_com_offset)) > 0.0
        for name in cfg.joint_names:
            assert name in urdf_joints, f'{robot}: {name} missing from URDF'
            assert name in mapped, f'{robot}: {name} missing from joints_map'
        assert set(cfg.joint_names) == mapped
        assert (PKG / 'mujoco' / f'{robot}.xml').is_file()
        assert (PKG / 'config' / f'{robot}_lowcmd.yaml').is_file()


def test_default_angles_inside_urdf_limits_and_mjcf_has_joints():
    for robot in ('go2', 'b2'):
        cfg = DeployConfig.from_yaml(PKG / 'config' / f'{robot}_rl.yaml')
        urdf = ET.fromstring((PKG / 'urdf' / f'{robot}.urdf').read_text(encoding='utf-8'))
        limits = {}
        for joint in urdf.findall('joint'):
            limit = joint.find('limit')
            if joint.attrib.get('type') in ('revolute', 'continuous') and limit is not None:
                limits[joint.attrib['name']] = (
                    float(limit.attrib['lower']), float(limit.attrib['upper']))
        for name, angle in zip(cfg.joint_names, cfg.default_angles):
            lo, hi = limits[name]
            assert lo <= float(angle) <= hi, f'{robot} {name}={angle} outside [{lo}, {hi}]'
        for mesh in urdf.iter('mesh'):
            filename = mesh.attrib.get('filename', '')
            assert 'champ' not in filename
            if filename.startswith('package://unitree_rl_deploy/'):
                rel = filename.split('package://unitree_rl_deploy/', 1)[1]
                assert (PKG / rel).is_file(), filename
        mjcf = ET.parse(PKG / 'mujoco' / f'{robot}.xml').getroot()
        joint_names = {el.attrib['name'] for el in mjcf.iter('joint') if 'name' in el.attrib}
        actuator_names = {el.attrib['name'] for el in mjcf.iter('position') if 'name' in el.attrib}
        body_names = {el.attrib['name'] for el in mjcf.iter('body') if 'name' in el.attrib}
        for name in cfg.joint_names:
            assert name in joint_names
            assert name in actuator_names
        for el in mjcf.iter('position'):
            if el.attrib.get('name') in cfg.joint_names:
                assert float(el.attrib['kp']) == pytest.approx(float(cfg.kp))
                assert float(el.attrib['kv']) == pytest.approx(float(cfg.kd))
        links = yaml.safe_load((PKG / 'config' / f'{robot}_links.yaml').read_text())
        links_map = links['/**']['ros__parameters']['links_map']
        assert links_map['imu'] in body_names
        assert links_map['base'] in body_names


def test_dls_blind_layout_is_default_260():
    cfg = DeployConfig.from_yaml(PKG / 'config' / 'go2_rl.yaml')
    assert cfg.num_obs == 260
    assert cfg.observation.terms == [
        'lin_vel', 'ang_vel', 'gravity', 'command', 'dof_pos', 'dof_vel', 'action', 'clock']
    assert cfg.observation.lin_vel_scale == 1.0
    assert cfg.cmd_timeout_sec == 0.5
    assert cfg.lin_vel_timeout_sec == 0.2
    assert cfg.kp == pytest.approx(20.0)
    assert cfg.kd == pytest.approx(2.0)
    b2 = DeployConfig.from_yaml(PKG / 'config' / 'b2_rl.yaml')
    assert b2.num_obs == 260
    assert b2.kp == pytest.approx(100.0)
    assert b2.kd == pytest.approx(5.0)
    data = yaml.safe_load((PKG / 'config' / 'go2_rl.yaml').read_text())
    params = dict(data['/**']['ros__parameters'])
    params['observation'] = dict(params['observation'])
    params['observation']['history'] = 1
    params['observation']['terms'] = [
        'ang_vel', 'gravity', 'command', 'dof_pos', 'dof_vel', 'action']
    without_lin_vel = DeployConfig.from_mapping(params)
    assert without_lin_vel.num_obs == 45
    assert without_lin_vel.observation.needs_lin_vel is False


def test_lin_vel_without_velocity_source_is_rejected():
    data = yaml.safe_load((PKG / 'config' / 'go2_rl.yaml').read_text())
    params = dict(data['/**']['ros__parameters'])
    params['odom_topic'] = ''
    params['base_state_topic'] = ''
    with pytest.raises(ValueError, match='lin_vel'):
        DeployConfig.from_mapping(params)


def test_invalid_lin_vel_frame_is_rejected():
    data = yaml.safe_load((PKG / 'config' / 'go2_rl.yaml').read_text())
    params = dict(data['/**']['ros__parameters'])
    params['lin_vel_frame'] = 'imu'
    with pytest.raises(ValueError, match='lin_vel_frame'):
        DeployConfig.from_mapping(params)


def test_parse_ros_bool_does_not_treat_false_string_as_true():
    assert parse_ros_bool('false') is False
    assert parse_ros_bool('true') is True
    assert parse_ros_bool(False) is False
    assert parse_ros_bool(True) is True
    assert parse_ros_bool(None, default=False) is False
    assert parse_ros_bool('0') is False
    assert parse_ros_bool('1') is True


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


def test_sim2real_launch_defaults_to_muse_base_state():
    text = (PKG / 'launch' / 'unitree_rl_sim2real.launch.py').read_text(encoding='utf-8')
    assert "DEFAULT_MUSE_BASE_STATE = '/base_state'" in text
    assert 'default_value=DEFAULT_MUSE_BASE_STATE' in text
    assert 'ros2 launch state_estimator state_estimator.launch.py' in text
    mujoco = (PKG / 'launch' / 'mujoco_rl.launch.py').read_text(encoding='utf-8')
    assert "default_value='odom/ground_truth'" in mujoco
    root = PKG.parents[1]
    muse_helper = (root / 'run_muse.sh').read_text(encoding='utf-8')
    assert 'state_estimator.launch.py' in muse_helper
    sim2real = (root / 'run_rl_sim2real.sh').read_text(encoding='utf-8')
    assert 'MUSE_WS' in sim2real
    assert 'dls2_interface' in sim2real
