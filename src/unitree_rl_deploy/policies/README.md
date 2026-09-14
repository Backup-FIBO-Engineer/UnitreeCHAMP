# Policies (not in git)

Drop a trained actor here. Training code is not part of this package.

Supported files:

- TorchScript export (typical Unitree `policy_1.pt` / `motion.pt`)
- PyTorch `state_dict` with `actor.*` Linear layers (legged_gym / unitree_rl_gym)
- ONNX (`.onnx`)

Launch:

```bash
ros2 launch unitree_rl_deploy mujoco_rl.launch.py robot:=go2 \
  policy:=/absolute/path/to/policy.pt
```

`policy_path` in `config/<robot>_rl.yaml` may be relative to this folder.
Leave it empty to hold `default_angles` (stand) so you can check topics
before the checkpoint is on disk.

The observation vector, joint order and `default_angles` in that yaml **must**
match the training env. The shipped yaml is 48-D (`lin_vel` first). A mismatch
walks backward, jerks, or falls.
