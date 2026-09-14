# Policies (not in git)

Drop a trained DLS Flat / Rough-Blind actor here. Training code is not part
of this package. Vision / height-map checkpoints are not supported yet.

Supported files:

- DLS ONNX export (`exported/policy.onnx`)
- TorchScript export
- PyTorch `state_dict` with `actor.*` Linear layers
- ONNX (`.onnx`)

The shipped yaml is **260-D** (52 × 5). The file must accept 260 floats and
return 12 actions. Joint order is DLS hip-group (FL/FR/RL/RR hips, then
thighs, then calves).

Launch:

```bash
ros2 launch unitree_rl_deploy mujoco_rl.launch.py robot:=go2 \
  policy:=/absolute/path/to/policy.pt
```

`policy_path` in `config/<robot>_rl.yaml` may be relative to this folder.
Leave it empty to hold `default_angles` (stand) so you can check topics
before the checkpoint is on disk.

A size or joint-order mismatch walks backward, jerks, or falls.
