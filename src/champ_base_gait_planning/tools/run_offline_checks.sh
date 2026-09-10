#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CHAMP_INCLUDE="${ROOT}/../champ/include/champ"
cd "$ROOT"

python3 tools/verify_urdf_champ_contract.py \
  urdf/xgo_rviz.xacro config/xgo_links.yaml config/xgo_joints.yaml
python3 tools/verify_urdf_champ_contract.py \
  urdf/go2.urdf config/go2_links.yaml config/go2_joints.yaml --preset go2
python3 tools/verify_urdf_champ_contract.py \
  urdf/b2.urdf config/b2_links.yaml config/b2_joints.yaml --preset b2
python3 tools/verify_gait_yaml.py go2
python3 tools/verify_gait_yaml.py b2
python3 tools/validate_mujoco_against_urdf.py urdf/xgo_rviz.xacro mujoco/xgo.xml
python3 tools/validate_mujoco_against_urdf.py urdf/go2.urdf mujoco/go2.xml --robot go2
python3 tools/validate_mujoco_against_urdf.py urdf/b2.urdf mujoco/b2.xml --robot b2
python3 tools/verify_mujoco_physics.py mujoco/xgo.xml
python3 tools/verify_mujoco_physics.py mujoco/go2.xml
python3 tools/verify_mujoco_physics.py mujoco/b2.xml
python3 tools/verify_mujoco_walk.py --robot go2 0.35
python3 tools/verify_mujoco_walk.py --robot b2 0.35

g++ -std=c++17 -O2 -I "$CHAMP_INCLUDE" -o /tmp/verify_champ_xgo tools/verify_champ_xgo.cpp
/tmp/verify_champ_xgo
g++ -std=c++17 -O2 -I "$CHAMP_INCLUDE" -o /tmp/verify_champ_go2 tools/verify_champ_go2.cpp
/tmp/verify_champ_go2
g++ -std=c++17 -O2 -I "$CHAMP_INCLUDE" -o /tmp/verify_champ_b2 tools/verify_champ_b2.cpp
/tmp/verify_champ_b2
g++ -std=c++17 -O2 -I include -o /tmp/verify_unitree_lowcmd \
  tools/verify_unitree_lowcmd.cpp src/motor_crc.cpp
/tmp/verify_unitree_lowcmd
python3 - <<'PY'
from pathlib import Path
fails = 0
for robot, mode, hexmode in (("go2", "1", "0x01"), ("b2", "10", "0x0A")):
    text = Path(f"config/{robot}_lowcmd.yaml").read_text()
    ok = f"motor_mode: {mode}" in text and f"robot: {robot}" in text
    print(f"[{'PASS' if ok else 'FAIL'}] {robot}_lowcmd robot={robot}, motor_mode {hexmode} (sdk2 stand example)")
    fails += 0 if ok else 1
raise SystemExit(1 if fails else 0)
PY

echo "All offline checks passed."
