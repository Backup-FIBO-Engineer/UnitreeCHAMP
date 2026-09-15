# Policies (not in git)

Drop a trained **Go2 Rough-Blind** actor here. Training is not in this
package. Use `Locomotion-Go2-Rough-Blind` export only — not Flat, not
Rough-Vision / height-map checkpoints.

Supported files:

- DLS ONNX export (`exported/policy.onnx`)
- TorchScript export (`exported/policy.pt`)
- PyTorch `state_dict` with `actor.*` Linear layers

The shipped yaml is **260-D** (52 × 5). The file must accept 260 floats and
return 12 actions. Joint order is DLS hip-group (FL/FR/RL/RR hips, then
thighs, then calves).

```bash
ros2 launch unitree_rl_deploy mujoco_rl.launch.py robot:=go2 \
  policy:=/absolute/path/to/policy.onnx
```

Empty `policy:=` holds `default_angles` (stand) so you can check topics
before the checkpoint is on disk.

A size or joint-order mismatch walks backward, jerks, or falls.
