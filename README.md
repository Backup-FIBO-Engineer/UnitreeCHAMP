# UnitreeRLDeploy — Go2 Rough-Blind (Train → Sim2Sim MuJoCo → Sim2Real)

Repo นี้เป็น **ตัวรัน policy ที่เทรนแล้ว** สำหรับ Unitree **Go2** งาน **`Locomotion-Go2-Rough-Blind`** เท่านั้น

- **ไม่มีโค้ดเทรน RL** ใน repo นี้ — เทรนที่ [iit-DLSLab/basic-locomotion-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-isaaclab) แล้วนำ `policy.onnx` มาใส่
- **Sim2Sim:** MuJoCo ใน repo นี้ (`./run_mujoco_rl.sh`)
- **Sim2Real:** `policy_runner` + `unitree_ros2_bridge` (`/lowcmd`) **ต้องใช้ MUSE** ประมาณ `/base_state`
- ไม่รวม B2, Flat, Rough-Vision, กล้อง Depth, CHAMP

```
Train (Isaac Lab, ภายนอก)          Deploy (repo นี้)
simulator GT lin_vel               Sim2Sim MuJoCo: odom/ground_truth
ไม่เปิด MUSE                       Sim2Real: MUSE /base_state  ← บังคับ
        │
        ▼
  policy.onnx  (260 → 12)
        │
        ├─► MuJoCo  /cmd_vel + IMU + joints + GT odom → policy_runner → joint_commands
        │
        └─► หุ่นจริง
              unitree_ros2_bridge  (/lowstate → joints/IMU, joint_commands → /lowcmd)
              MUSE                 (/lowstate → /base_state)
              policy_runner        (/base_state + joints + /cmd_vel → joint_commands)
              Xbox                 (/cmd_vel)
```

`policy_runner` **ไม่ได้เปิด MUSE ให้เอง** และ **ไม่ได้ subscribe** `/odom` หรือ `/dog_imu_raw` เป็นค่าเริ่มต้นตอน Sim2Real

---

## 1. Observation ที่ต้องเหมือนตอน Train

Task ค่าเริ่มต้น:

```
use_imu = False
use_concurrent_state_est = False
use_rma = False
use_observation_history = True   # 5 เฟรม
use_clock_signal = True          # gait clock 4 ค่า
use_filter_actions = True        # α = 0.8 ที่เป้าข้อต่อ
```

หนึ่งเฟรม **52** ค่า เรียงเก่า → ใหม่ **5** เฟรม = **260** ค่าเข้า actor  
Train: ประมาณ `[4096, 260]` — `--num_envs=4096` เพิ่มจำนวนหุ่นที่ฝึก **ไม่ได้** เพิ่ม observation ต่อตัว  
Deploy: `[1, 260]`  
ความถี่ policy **50 Hz** (`dt = 0.005 s` × `decimation = 4`)

| Observation | จำนวน | ตอน Train (Isaac Lab) | ตอน Sim2Sim MuJoCo (repo นี้) | ตอน Sim2Real |
|---|---:|---|---|---|
| Base linear velocity XYZ | 3 | `root_lin_vel_b` จาก simulator (กรอบลำตัว, COM) | `odom/ground_truth` แล้วบวก `ω × r_com` | MUSE `/base_state` ความเร็วเชิงเส้น **world** → หมุนเป็น body |
| Base angular velocity XYZ | 3 | `root_ang_vel_b` | IMU จาก MuJoCo | `velocity.angular` จาก `/base_state` (body) |
| Projected gravity | 3 | `projected_gravity_b` | quaternion จาก IMU จำลอง | quaternion จาก `/base_state` |
| Velocity command | 3 | สุ่มตอนเทรน | `/cmd_vel` (Xbox หรือ `ros2 topic pub`) | `/cmd_vel` จาก Xbox |
| Joint position − default | 12 | มุมจำลอง − มุมอ้างอิง | `/joint_states` − `default_angles` | `/joint_states` จาก bridge (`/lowstate`) |
| Joint velocity | 12 | ความเร็วข้อต่อจำลอง | `/joint_states` | `/joint_states` |
| Previous action | 12 | action ล่าสุดใน env | wrapper เก็บเอง | wrapper เก็บเอง |
| Gait phase / clock | 4 | จาก phase + คำสั่งความเร็ว | พารามิเตอร์เดียวกับ training yaml | เช่นเดียวกับ Sim2Sim |
| **รวมต่อเฟรม** | **52** | | | |

**ห้าม** เอา `linear_acceleration` จาก IMU มาใส่ 3 ช่องแรก แม้จะมี 3 ค่าเท่ากัน — ความหมายและหน่วยต่างกัน  
ถ้าเทรนด้วย `use_imu=True` โค้ดต้นทางจะเปลี่ยน 3 ช่องแรกเป็นความเร่งจริง ๆ จึงใช้ policy ชุดนี้ไม่ได้

### Critic (ใช้ตอนเทรนเท่านั้น)

`use_asymmetric_ppo=True`: critic ได้ข้อมูลพิเศษ (contact, เวลาเท้าลอย, ความสูงพื้น ฯลฯ)  
**ตอนรัน actor บน MuJoCo / หุ่นจริงไม่ต้องมีกล้องหรือ heightmap**

### ลำดับข้อต่อ (hip-group ของ DLS)

อย่าส่ง array จาก Unitree SDK ตามลำดับขาโดยไม่จัดใหม่

```
FL_hip  FR_hip  RL_hip  RR_hip
FL_thigh FR_thigh RL_thigh RR_thigh
FL_calf  FR_calf  RL_calf  RR_calf
```

`default_angles`: hip `0`, thigh `0.9`, calf `−1.8`  
`action_scale = 0.5`, clip `±3`, PD เดิน Go2 **kp 20 / kd 2**  
คำสั่งสูงสุด `max_cmd = [0.8, 0.25, 0.5]` (vx, vy, yaw)

ไฟล์ที่ยึด layout นี้: `src/unitree_rl_deploy/config/go2_rl.yaml`

อ้างอิงต้นทางที่ตรวจ observation:

- [locomotion_env.py @ dffc687](https://github.com/iit-DLSLab/basic-locomotion-isaaclab/blob/dffc6874196f70afe1a04308994e34a9db079ffe/source/basic_locomotion_isaaclab/basic_locomotion_isaaclab/tasks/locomotion/locomotion_env.py)
- [go2_env_cfg.py @ dffc687](https://github.com/iit-DLSLab/basic-locomotion-isaaclab/blob/dffc6874196f70afe1a04308994e34a9db079ffe/source/basic_locomotion_isaaclab/basic_locomotion_isaaclab/tasks/locomotion/go2_env_cfg.py)
- [run_controller_ros2.py @ dffc687](https://github.com/iit-DLSLab/basic-locomotion-isaaclab/blob/dffc6874196f70afe1a04308994e34a9db079ffe/deploy/run_controller_ros2.py)

---

## 2. Train (ทำนอก repo นี้ — README อย่างเดียว)

MUSE **ไม่ได้ใช้ตอนเทรนใน Isaac Lab** ตัวจำลองให้ `root_lin_vel_b` โดยตรง  
MUSE ใช้ตอน **Sim2Real** (และถ้าใช้สแตก ROS 2 ของ DLS บนหุ่นจริง)

### 2.1 ติดตั้ง Isaac Lab + DLS

ตาม [README_train.md](https://github.com/iit-DLSLab/basic-locomotion-isaaclab/blob/main/README_train.md)

```bash
# 1) ติดตั้ง Isaac Lab ตามคู่มือต้นทาง (แนะนำ conda)
# 2) git-lfs
sudo apt install git-lfs

# 3) clone นอกโฟลเดอร์ IsaacLab
git clone https://github.com/iit-DLSLab/basic-locomotion-isaaclab.git
cd basic-locomotion-isaaclab

# 4) interpreter เดียวกับ Isaac Lab
python -m pip install -e source/basic_locomotion_isaaclab
```

ตรวจเวอร์ชัน Isaac Lab / rsl-rl ที่ README ต้นทางระบุว่าทดสอบแล้ว

### 2.2 เทรน Rough-Blind

```bash
python scripts/rsl_rl/train.py \
  --task=Locomotion-Go2-Rough-Blind \
  --num_envs=4096 \
  --headless \
  --logger=tensorboard
```

อย่าเปลี่ยนเป็น `Locomotion-Go2-Flat` หรือ `Locomotion-Go2-Rough-Vision` สำหรับสแตกนี้  
Rough-Vision ค่าเริ่มต้นเป็น heightmap จาก RayCaster และ `use_depth_camera = False` — **ไม่ใช่** Depth-to-Action

ดู TensorBoard จาก `logs/rsl_rl/...`

### 2.3 เล่นใน Isaac + export actor

`play.py` จะพยายามเขียน `exported/policy.onnx` และ `exported/policy.pt` ข้าง checkpoint

```bash
python scripts/rsl_rl/play.py \
  --task=Locomotion-Go2-Rough-Blind \
  --num_envs=16 \
  --visualizer newton
```

หรือชี้ checkpoint ตรง ๆ:

```bash
python scripts/rsl_rl/play.py \
  --task=Locomotion-Go2-Rough-Blind \
  --checkpoint=/abs/path/to/model_XXXX.pt
```

ไฟล์ที่ต้องคัดลอกมา repo นี้:

| ไฟล์ | ใช้ทำอะไร |
|---|---|
| `logs/.../exported/policy.onnx` (หรือ `policy.pt`) | actor 260 → 12 |
| `logs/.../params/env.yaml` | ตรวจว่า observation / default pose ตรง `go2_rl.yaml` |

วาง policy ที่ `src/unitree_rl_deploy/policies/` หรือส่ง `policy:=/abs/path/policy.onnx` ตอน launch  
`deploy/config.py` ของ DLS ค่าเริ่มต้นชี้ `tested_policies/go2/front_heightmap` — **อย่าใช้ไฟล์นั้นกับ Rough-Blind**

---

## 3. ติดตั้ง repo นี้ (deploy อย่างเดียว)

```bash
source /opt/ros/humble/setup.bash

# ข้อความ /lowcmd /lowstate สำหรับฮาร์ดแวร์
git clone https://github.com/unitreerobotics/unitree_ros2 ~/unitree_ros2
cd ~/unitree_ros2
git submodule update --init --recursive
cd cyclonedds_ws
colcon build --packages-select unitree_go unitree_api
source install/setup.bash

# ที่ราก workspace นี้
./run_build_ws.sh
source install/setup.bash
python3 -m pytest src/unitree_rl_deploy/test -q
```

Inference: **PyTorch** สำหรับ `.pt` หรือ **onnxruntime** สำหรับ `.onnx`

```bash
pip install torch onnxruntime   # เลือกตามที่เครื่องมี CUDA หรือไม่
```

---

## 4. Sim2Sim MuJoCo (ไม่ใช้ MUSE)

ในเส้นนี้ simulator ให้ IMU, ข้อต่อ และ `odom/ground_truth` (กรอบลำตัว) ครบ  
**ไม่ต้อง** เปิด `state_estimator.launch.py`

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

./run_mujoco_rl.sh go2 policy:=/abs/path/to/policy.onnx
```

ไม่มี `policy:=` หุ่นจะยืนที่ `default_angles` เพื่อตรวจ topic

อีกเทอร์มินัล — คำสั่งความเร็ว:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

หรือ Xbox:

```bash
./run_xbox_teleop.sh go2
```

ถือ **RB** ค้างแล้วดันสติกซ้ายเดิน (ปล่อย RB = หยุด)

ตรวจ:

```bash
ros2 topic echo /rl/observation    # ควรยาว 260
ros2 topic echo /rl/action         # 12
ros2 topic echo /joint_commands
ros2 topic echo /cmd_vel
```

ทางเลือกต้นทาง DLS (ไม่จำเป็นถ้าใช้ MuJoCo ใน repo นี้): ตั้งหุ่น/policy ใน `deploy/config.py` แล้ว `python3 deploy/play_mujoco.py` ตาม [README_deploy.md](https://github.com/iit-DLSLab/basic-locomotion-isaaclab/blob/main/README_deploy.md)

---

## 5. MUSE — บังคับสำหรับ Sim2Real

[MUSE branch `unitree_sdk`](https://github.com/iit-DLSLab/muse/tree/unitree_sdk) ประมาณสถานะจาก **IMU + ข้อต่อ + แรงที่เท้า** บน `/lowstate`  
DLS ระบุว่าทดสอบกับ **Go2** ทั้งซิมและหุ่นจริงแล้ว  
**ไม่ได้รวมซอร์ส MUSE ใน repo นี้** ติดตั้งแยก แล้วเปิดคนละเทอร์มินัลกับ RL controller

ค่าเริ่มต้นของ plugin เขียน `base_state_topic: /base_state` และอ่าน `/lowstate`

### 5.1 ติดตั้ง MUSE

```bash
git clone https://github.com/iit-DLSLab/muse.git -b unitree_sdk
cd muse
conda env create -f environment.yml
conda activate muse-ros2
cd muse_ws
vcs import src < muse.repos
colcon build --symlink-install
source install/setup.bash
```

รายละเอียด: [MUSE README — Building and Running](https://github.com/iit-DLSLab/muse/tree/unitree_sdk#-building-and-running)

workspace ของ MUSE มี `dls2_interface` (ชนิดข้อความ `BaseState`)  
ตอนรัน `policy_runner` ต้อง **source MUSE ก่อน แล้วค่อย source overlay ของ repo นี้** เพื่อ import `dls2_interface` ได้  
`./run_rl_sim2real.sh` จะ source `$MUSE_WS` (ค่าเริ่มต้น `~/muse/muse_ws`) ให้อัตโนมัติถ้า build แล้ว

### 5.2 เปิด estimator

ทุกเทอร์มินัลที่คุยกับหุ่นต้องใช้ CycloneDDS โดเมนเดียวกับหุ่น (ดูหัวข้อ 6)

```bash
# จากราก repo นี้ หลัง source ROS + CycloneDDS แล้ว
./run_muse.sh

# หรือ
source ~/muse/muse_ws/install/setup.bash
ros2 launch state_estimator state_estimator.launch.py
```

ตรวจว่ามี `/base_state`:

```bash
ros2 topic list | grep base_state
ros2 topic echo /base_state --once
```

ถ้ายังไม่มี `/base_state` และ `/joint_states` ครบ `policy_runner` **จะรอและส่งแค่ท่า default_angles ไม่คำนวณ RL**

MUSE ใช้ IMU ภายใน estimator ได้ — ค่า `use_imu=False` ของ task หมายถึง **policy ไม่กินความเร่ง** ไม่ได้ห้าม MUSE ใช้ IMU

อย่าเปิด Point-LIO / LiDAR fusion สำหรับ Rough-Blind ถ้ายังไม่ตั้งค่า — ใช้ proprioceptive ตาม `state_estimator.launch.py` พอ

---

## 6. Sim2Real บน Go2

### 6.1 เครือข่ายและ Sport

1. สาย LAN ไปหุ่น ตั้ง IPv4 เครื่องเป็น `192.168.123.99/24` (Go2)
2. ปิด **Sport** / โหมดแอปอย่างเป็นทางการ — สแตกนี้ยิง `/lowcmd` เอง อย่าผสม Sport API
3. CycloneDDS โดเมน 0 ผูก NIC ที่ต่อหุ่น (เช่น `eth0`, `enp3s0`)

`./run_rl_sim2real.sh <nic> ...` ตั้ง `RMW_IMPLEMENTATION` และ `CYCLONEDDS_URI` ให้ทุกโหนดใน launch นั้น  
เทอร์มินัล MUSE ต้องเห็น `/lowstate` ด้วย — ใช้ NIC เดียวกัน:

```bash
NETWORK_INTERFACE=eth0 ./run_muse.sh
```

หรือ source สคริปต์เชื่อมต่อของ [unitree-ros2-dls](https://github.com/iit-DLSLab/unitree-ros2-dls) (`unitree_ros2_connect.bash`) ใน **ทุก** เทอร์มินัล ตาม [README_unitree_ros2.md](https://github.com/iit-DLSLab/unitree-ros2-dls/blob/main/README_unitree_ros2.md)

HAL ของ DLS (`python3 launch_go2_hal.py`) เป็นทางเลือกถ้าใช้สแตก DLS ทั้งชุด  
repo นี้ใช้ `unitree_ros2_bridge` แทน HAL สำหรับ `/lowcmd` + `/lowstate` → `joint_states` / `imu/data`

### 6.2 เปิดทีละเทอร์มินัล (ลำดับที่แนะนำ)

**เทอร์มินัล A — MUSE** (ต้องเห็น `/lowstate`)

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash   # ถ้ามี
NETWORK_INTERFACE=eth0 ./run_muse.sh
```

**เทอร์มินัล B — bridge + policy**

```bash
./run_rl_sim2real.sh eth0 policy:=/abs/path/to/policy.onnx
```

เทียบเท่า:

```bash
source /opt/ros/humble/setup.bash
source ~/muse/muse_ws/install/setup.bash
source install/setup.bash
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py \
  robot:=go2 \
  network_interface:=eth0 \
  policy:=/abs/path/to/policy.onnx
```

ค่าเริ่มต้น Sim2Real: `base_state_topic:=/base_state`, ไม่ใช้ `/odom`  
ถ้ายังไม่มี `dls2_interface` โหนดจะ **ล้มทันที** พร้อมข้อความให้ source MUSE — ไม่ได้เติมศูนย์ในช่องความเร็ว

**เทอร์มินัล C — Xbox `/cmd_vel`**

```bash
./run_xbox_teleop.sh go2
```

ถือ RB แล้วเดิน จำกัดความเร็วตาม `max_cmd` ใน `go2_rl.yaml`  
อย่ารัน `teleop_twist_keyboard` พร้อมกัน (ชน `/cmd_vel`)

### 6.3 ตรวจก่อนปล่อยเดิน

```bash
ros2 topic hz /lowstate
ros2 topic hz /base_state
ros2 topic hz /joint_states
ros2 topic echo /rl/observation --once    # len = 260
ros2 topic echo /joint_commands --once
```

log ของ `policy_runner` ต้องไม่ค้างที่ `waiting for lin_vel from MUSE /base_state`  
เมื่อ MUSE + ข้อต่อมาแล้ว จึงเริ่มออกจากท่า stand

---

## 7. Xbox (สรุป)

แพดเกม Xbox One S (1708) Bluetooth — รายละเอียดที่ `src/xbox_one_s_teleop/README.md`

| ปุ่ม | หน้าที่ |
|---|---|
| **RB ค้าง** | เปิด `/cmd_vel` (ปล่อย = หยุด) |
| **B** | ศูนย์ทันที |
| สติกซ้าย | vx / vy |
| สติกขวา X | เลี้ยว |

---

## 8. สิ่งที่ต้องตรงก่อนขึ้นหุ่นจริง

1. ใช้ `policy.onnx` (หรือ `.pt`) **จากรัน Rough-Blind เดียวกัน** ไม่ใช่ policy heightmap ของ DLS
2. ลำดับข้อต่อ hip-group ตามตารางด้านบน
3. `default_angles` ตรงท่ายืนตอนเทรน ถ้าเปลี่ยนท่ายืนตอนเทรน ต้องแก้ `go2_rl.yaml`
4. เปิด **MUSE** คนละโปรเซส — controller รอ `/base_state`
5. PD เดิน 20/2 ถูกส่งจาก yaml ไป bridge
6. ช่องว่าง Train ↔ หุ่นจริงอยู่ที่ความคลาดของ state estimation, สัญญาณรบกวน, ความหน่วง — ตอนเทรนมี observation noise ตอน deploy ใช้ค่าเซนเซอร์จริงโดยไม่เติม noise จำลอง

เอกสารว่ามี Sim2Real **ไม่เท่ากับ** ว่าทดสอบบนหุ่น Go2 เครื่องนี้แล้ว

---

## 9. แพ็กเกจใน workspace

| แพ็กเกจ | หน้าที่ |
|---|---|
| `src/unitree_rl_deploy` | policy runner, MuJoCo, `/lowcmd` bridge, URDF Go2 |
| `src/xbox_one_s_teleop` | Xbox → `/cmd_vel` |
| `src/unitree_interfaces` | ข้อความ `unitree_go` / `unitree_api` (หรือ source จาก unitree_ros2) |

สคริปต์ราก repo:

| สคริปต์ | ใช้เมื่อ |
|---|---|
| `./run_build_ws.sh` | build overlay |
| `./run_mujoco_rl.sh` | Sim2Sim |
| `./run_muse.sh` | เปิด MUSE (ต้อง build `muse_ws` ก่อน) |
| `./run_rl_sim2real.sh <nic>` | Sim2Real (source MUSE ถ้ามี) |
| `./run_xbox_teleop.sh go2` | สติก |

---

## 10. แก้ปัญหาบ่อย

| อาการ | สาเหตุที่พบบ่อย |
|---|---|
| ค้างท่า stand, log `waiting for lin_vel from MUSE` | ยังไม่เปิด MUSE / CycloneDDS คนละ NIC / ไม่มี `/base_state` |
| `dls2_interface is not on PYTHONPATH` | ลืม `source ~/muse/muse_ws/install/setup.bash` ก่อน overlay นี้ |
| เดินถอยหลังทั้งที่สติกไปข้างหน้า | เครื่องหมาย `/cmd_vel` — ลอง `invert_vx:=true` ที่ Xbox |
| กระตุก / ล้มทันที | ลำดับข้อต่อหรือ `default_angles` ไม่ตรงเทรน หรือป้อน policy คนละ task |
| IMU timeout บน MuJoCo | ซิมยังไม่สเต็ป — รอ `joint_commands` รอบแรก (ส่ง default_angles ให้) |
| MUSE ไม่ขึ้น | ไม่มี `/lowstate` จากหุ่น หรือยังไม่ source CycloneDDS ในเทอร์มินัล MUSE |

---

## 11. ลิงก์ต้นทาง

- Train / task: [basic-locomotion-isaaclab](https://github.com/iit-DLSLab/basic-locomotion-isaaclab)
- Deploy ต้นทาง DLS: [README_deploy.md](https://github.com/iit-DLSLab/basic-locomotion-isaaclab/blob/main/README_deploy.md)
- HAL Unitree (ทางเลือก): [unitree-ros2-dls](https://github.com/iit-DLSLab/unitree-ros2-dls)
- State estimator: [muse `unitree_sdk`](https://github.com/iit-DLSLab/muse/tree/unitree_sdk)
- DDS หุ่น: [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
