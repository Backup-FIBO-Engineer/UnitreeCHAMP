# unitree_rl_deploy

รัน policy **Go2 Rough-Blind** ที่เทรนแล้วใน MuJoCo และบนหุ่นจริง  
คู่มือเต็ม (Train → export → Sim2Sim → MUSE → Sim2Real) อยู่ที่ [README.md](../../README.md) ราก repo

ไม่มีโค้ดเทรนในแพ็กเกจนี้ Observation actor คือ **260-D** (52 × 5) ตาม `config/go2_rl.yaml`

```
Sim2Sim:  /cmd_vel + IMU + joints + odom/ground_truth → policy_runner → joint_commands → MuJoCo
Sim2Real: /cmd_vel + joints + MUSE /base_state         → policy_runner → joint_commands → /lowcmd
```

Xbox: `./run_xbox_teleop.sh go2`

## ไฟล์สำคัญ

| ไฟล์ | หน้าที่ |
|---|---|
| `config/go2_rl.yaml` | Observation, `default_angles`, ลำดับข้อต่อ, `action_scale`, PD |
| `policies/` | `.onnx` / `.pt` ของคุณ (ไม่เก็บใน git) |
| `launch/mujoco_rl.launch.py` | Sim2Sim |
| `launch/unitree_rl_sim2real.launch.py` | Sim2Real ค่าเริ่มต้น `base_state_topic:=/base_state` |

ลำดับข้อต่อ DLS hip-group:

```
FL_hip FR_hip RL_hip RR_hip | FL_thigh FR_thigh RL_thigh RR_thigh | FL_calf FR_calf RL_calf RR_calf
```

`lin_vel` คือความเร็วลำตัวกรอบ body **ห้าม** ใส่ IMU acceleration แทน

```bash
./run_mujoco_rl.sh go2 policy:=/abs/path/policy.onnx
./run_muse.sh
./run_rl_sim2real.sh eth0 policy:=/abs/path/policy.onnx
./run_xbox_teleop.sh go2
python3 -m pytest src/unitree_rl_deploy/test -q
```
