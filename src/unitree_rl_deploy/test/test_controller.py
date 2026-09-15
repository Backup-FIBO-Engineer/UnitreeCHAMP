import numpy as np
import pytest

from unitree_rl_deploy.controller import PolicyController, SensorSample
from unitree_rl_deploy.config import DeployConfig
from unitree_rl_deploy.policy import NumpyMlpPolicy, StandPolicy, load_policy


def _deploy_cfg(**overrides):
    data = {
        'joint_names': [f'j{i}' for i in range(12)],
        'default_angles': [0.1, 0.8, -1.5] * 4,
        'action_scale': 0.25,
        'hip_scale_reduction': 1.0,
        'hip_indices': [0, 3, 6, 9],
        'observation': {
            'terms': ['ang_vel', 'gravity', 'command', 'dof_pos', 'dof_vel', 'action'],
            'clip': 100.0,
            'history': 1,
            'cmd_scale': [1.0, 1.0, 1.0],
            'ang_vel_scale': 1.0,
            'dof_pos_scale': 1.0,
            'dof_vel_scale': 1.0,
        },
    }
    data.update(overrides)
    return DeployConfig.from_mapping(data)


def _level_sample(cfg, q=None):
    n = cfg.num_actions
    return SensorSample(
        q=np.asarray(cfg.default_angles if q is None else q, dtype=np.float32),
        dq=np.zeros(n, dtype=np.float32),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
        ang_vel=np.zeros(3, dtype=np.float32),
        cmd_vx=0.0,
        cmd_vy=0.0,
        cmd_wz=0.0,
        orientation_valid=True,
        time_sec=0.0,
    )


def test_stand_policy_holds_default_pose():
    cfg = _deploy_cfg()
    loop = PolicyController(cfg, StandPolicy(cfg.num_actions))
    targets = loop.targets(_level_sample(cfg))
    np.testing.assert_allclose(targets, cfg.default_angles, atol=1e-6)
    np.testing.assert_allclose(loop.last_action, 0.0, atol=1e-6)


def test_constant_action_scales_onto_defaults():
    cfg = _deploy_cfg(action_scale=0.25, hip_scale_reduction=0.5)

    def policy(obs):
        del obs
        return np.ones(12, dtype=np.float32)

    loop = PolicyController(cfg, policy)
    targets = loop.targets(_level_sample(cfg))
    expected = np.array(cfg.default_angles, dtype=np.float32)
    expected += 0.25
    expected[[0, 3, 6, 9]] = np.array(cfg.default_angles, dtype=np.float32)[[0, 3, 6, 9]] + 0.125
    np.testing.assert_allclose(targets, expected, atol=1e-5)


def test_cmd_vel_is_clamped_before_obs():
    cfg = _deploy_cfg(max_cmd=[0.5, 0.2, 0.4])
    seen = {}

    def policy(obs):
        seen['obs'] = np.asarray(obs, dtype=np.float32).copy()
        return np.zeros(12, dtype=np.float32)

    loop = PolicyController(cfg, policy)
    sample = _level_sample(cfg)
    sample.cmd_vx = 9.0
    sample.cmd_vy = -9.0
    sample.cmd_wz = 9.0
    loop.targets(sample)
    # command occupies obs[6:9] with cmd_scale 1
    np.testing.assert_allclose(seen['obs'][6:9], [0.5, -0.2, 0.4], atol=1e-5)


def test_numpy_mlp_roundtrip():
    rng = np.random.default_rng(0)
    w1 = rng.normal(size=(8, 4)).astype(np.float32)
    b1 = rng.normal(size=(8,)).astype(np.float32)
    w2 = rng.normal(size=(2, 8)).astype(np.float32)
    b2 = rng.normal(size=(2,)).astype(np.float32)
    mlp = NumpyMlpPolicy([(w1, b1), (w2, b2)])
    x = rng.normal(size=(4,)).astype(np.float32)
    hidden = w1 @ x + b1
    hidden = np.where(hidden > 0.0, hidden, np.expm1(hidden))
    np.testing.assert_allclose(mlp(x), w2 @ hidden + b2, atol=1e-5)


def test_empty_policy_path_is_stand():
    policy = load_policy('', num_obs=45, num_actions=12)
    assert isinstance(policy, StandPolicy)
    np.testing.assert_allclose(policy(np.zeros(45)), 0.0)


def test_missing_policy_file():
    with pytest.raises(FileNotFoundError):
        load_policy('/no/such/policy.pt', num_obs=45, num_actions=12)


def test_obs_size_must_match_yaml():
    cfg = _deploy_cfg()
    assert cfg.num_obs == 45
    loop = PolicyController(cfg, lambda obs: np.zeros(11, dtype=np.float32))
    with pytest.raises(ValueError, match='11 actions'):
        loop.targets(_level_sample(cfg))


def test_controller_packs_48d_lin_vel_first():
    cfg = _deploy_cfg(observation={
        'terms': ['lin_vel', 'ang_vel', 'gravity', 'command', 'dof_pos', 'dof_vel', 'action'],
        'clip': 100.0,
        'history': 1,
        'cmd_scale': [1.0, 1.0, 1.0],
        'ang_vel_scale': 1.0,
        'dof_pos_scale': 1.0,
        'dof_vel_scale': 1.0,
        'lin_vel_scale': 2.0,
    })
    assert cfg.num_obs == 48
    seen = {}

    def policy(obs):
        seen['obs'] = np.asarray(obs, dtype=np.float32).copy()
        return np.zeros(12, dtype=np.float32)

    loop = PolicyController(cfg, policy)
    sample = _level_sample(cfg)
    sample.lin_vel = np.array([0.5, -0.1, 0.0], dtype=np.float32)
    loop.targets(sample)
    np.testing.assert_allclose(seen['obs'][:3], [1.0, -0.2, 0.0], atol=1e-5)
    assert seen['obs'].size == 48


def test_controller_rejects_48d_without_lin_vel_sample():
    cfg = _deploy_cfg(observation={
        'terms': ['lin_vel', 'ang_vel', 'gravity', 'command', 'dof_pos', 'dof_vel', 'action'],
        'clip': 100.0,
        'history': 1,
    })
    loop = PolicyController(cfg, lambda obs: np.zeros(12, dtype=np.float32))
    with pytest.raises(ValueError, match='lin_vel'):
        loop.targets(_level_sample(cfg))
