"""Load a trained actor. Training loops are not implemented here."""
from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional, Sequence, Union

import numpy as np

PolicyFn = Callable[[np.ndarray], np.ndarray]


class StandPolicy:
    """Zeros: target joints stay at default_angles. Used when no checkpoint is set."""

    def __init__(self, num_actions: int) -> None:
        if num_actions <= 0:
            raise ValueError('num_actions must be > 0')
        self.num_actions = int(num_actions)

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        del observation
        return np.zeros(self.num_actions, dtype=np.float32)


class NumpyMlpPolicy:
    """ELU MLP whose weights are (W, b) pairs, input → hidden… → actions."""

    def __init__(self, layers: Sequence[tuple[np.ndarray, np.ndarray]]) -> None:
        if not layers:
            raise ValueError('MLP needs at least one layer')
        self.layers = []
        for index, (weight, bias) in enumerate(layers):
            weight = np.asarray(weight, dtype=np.float32)
            bias = np.asarray(bias, dtype=np.float32).reshape(-1)
            if weight.ndim != 2 or weight.shape[0] != bias.shape[0]:
                raise ValueError(
                    f'layer {index}: weight {weight.shape} does not match bias {bias.shape}')
            self.layers.append((weight, bias))

    @property
    def num_obs(self) -> int:
        return int(self.layers[0][0].shape[1])

    @property
    def num_actions(self) -> int:
        return int(self.layers[-1][0].shape[0])

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        x = np.asarray(observation, dtype=np.float32).reshape(-1)
        if x.size != self.num_obs:
            raise ValueError(f'policy expected {self.num_obs} obs, got {x.size}')
        for index, (weight, bias) in enumerate(self.layers):
            x = weight @ x + bias
            if index < len(self.layers) - 1:
                x = np.where(x > 0.0, x, np.expm1(x))
        return x.astype(np.float32, copy=False)


def resolve_policy_path(path: Union[str, Path], search_dirs: Sequence[Union[str, Path]]) -> Path:
    configured = Path(str(path)).expanduser()
    if configured.is_file():
        return configured.resolve()
    if configured.is_absolute():
        raise FileNotFoundError(f'policy file not found: {configured}')
    for folder in search_dirs:
        candidate = Path(folder) / configured
        if candidate.is_file():
            return candidate.resolve()
    searched = ', '.join(str(Path(d)) for d in search_dirs)
    raise FileNotFoundError(
        f'policy file {configured} not found (looked in {searched} and as an absolute path)')


def load_policy(
    path: Union[str, Path, None],
    num_obs: int,
    num_actions: int,
    search_dirs: Optional[Sequence[Union[str, Path]]] = None,
) -> PolicyFn:
    """Load TorchScript, actor state_dict, ONNX, or a stand policy if path is empty."""
    if path is None or str(path).strip() in ('', 'stand', 'none'):
        return StandPolicy(num_actions)

    resolved = resolve_policy_path(path, search_dirs or ())
    suffix = resolved.suffix.lower()
    if suffix == '.onnx':
        return _load_onnx(resolved, num_obs, num_actions)
    if suffix in ('.pt', '.pth'):
        return _load_torch(resolved, num_obs, num_actions)
    raise ValueError(
        f'unsupported policy {resolved}; use TorchScript/state_dict .pt/.pth or ONNX .onnx')


def _load_onnx(path: Path, num_obs: int, num_actions: int) -> PolicyFn:
    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise ImportError(
            'ONNX policy requires the onnxruntime package. pip install onnxruntime'
        ) from exc
    session = ort.InferenceSession(str(path), providers=['CPUExecutionProvider'])
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    def infer(observation: np.ndarray) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32).reshape(1, -1)
        if obs.shape[1] != num_obs:
            raise ValueError(f'policy expected {num_obs} obs, got {obs.shape[1]}')
        out = session.run([output_name], {input_name: obs})[0]
        action = np.asarray(out, dtype=np.float32).reshape(-1)
        if action.size != num_actions:
            raise ValueError(f'policy returned {action.size} actions, expected {num_actions}')
        return action

    return infer


def _load_torch(path: Path, num_obs: int, num_actions: int) -> PolicyFn:
    try:
        import torch
    except ImportError as exc:
        raise ImportError(
            'PyTorch policy requires torch. pip install torch  (CPU is enough for deploy)'
        ) from exc

    try:
        module = torch.jit.load(str(path), map_location='cpu')
        module.eval()
        return _wrap_torch_module(module, num_obs, num_actions)
    except (RuntimeError, OSError, ValueError):
        pass

    try:
        obj = torch.load(str(path), map_location='cpu', weights_only=False)
    except TypeError:
        obj = torch.load(str(path), map_location='cpu')
    if hasattr(obj, '__call__') and not isinstance(obj, dict):
        if hasattr(obj, 'eval'):
            obj.eval()
        return _wrap_torch_module(obj, num_obs, num_actions)

    state = obj['model_state_dict'] if isinstance(obj, dict) and 'model_state_dict' in obj else obj
    if not isinstance(state, dict):
        raise ValueError(f'{path} is not TorchScript, a callable module, or a state_dict')
    mlp = _mlp_from_actor_state_dict(state, num_obs, num_actions)
    return mlp


def _wrap_torch_module(module, num_obs: int, num_actions: int) -> PolicyFn:
    import torch

    def infer(observation: np.ndarray) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32).reshape(1, -1)
        if obs.shape[1] != num_obs:
            raise ValueError(f'policy expected {num_obs} obs, got {obs.shape[1]}')
        with torch.no_grad():
            tensor = torch.from_numpy(obs)
            out = module(tensor)
            if isinstance(out, (tuple, list)):
                out = out[0]
            action = out.detach().cpu().numpy().reshape(-1)
        if action.size != num_actions:
            raise ValueError(f'policy returned {action.size} actions, expected {num_actions}')
        return action.astype(np.float32, copy=False)

    return infer


def _to_numpy(value) -> np.ndarray:
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float32)


def _mlp_from_actor_state_dict(state: dict, num_obs: int, num_actions: int) -> NumpyMlpPolicy:
    """legged_gym / unitree_rl_gym ActorCritic: Sequential Linear (+ELU) named actor.N."""
    import re

    indices = []
    for key in state:
        match = re.fullmatch(r'actor\.(\d+)\.weight', str(key))
        if match:
            indices.append(int(match.group(1)))
    indices.sort()
    if not indices:
        keys = ', '.join(sorted(str(k) for k in list(state.keys())[:12]))
        raise ValueError(
            'state_dict has no actor.N.weight layers (expected unitree_rl_gym / legged_gym '
            f'ActorCritic). First keys: {keys}')
    layers = []
    for index in indices:
        bias_key = f'actor.{index}.bias'
        if bias_key not in state:
            raise ValueError(f'state_dict has actor.{index}.weight but no {bias_key}')
        layers.append((_to_numpy(state[f'actor.{index}.weight']), _to_numpy(state[bias_key])))
    mlp = NumpyMlpPolicy(layers)
    if mlp.num_obs != num_obs:
        raise ValueError(f'actor input is {mlp.num_obs}, yaml observation size is {num_obs}')
    if mlp.num_actions != num_actions:
        raise ValueError(f'actor output is {mlp.num_actions}, yaml num_actions is {num_actions}')
    return mlp
