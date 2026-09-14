# Xbox One S Model 1708 (Bluetooth) teleop

ROS 2 Humble package (`xbox_one_s_teleop`) for this CHAMP workspace: one pad
for `/cmd_vel` and `/body_pose`. Pair with `joy_node` (`ros-humble-joy`). Stick
indices are `config/xbox_one_s_1708_bt.yaml` (SDL2). Speed and body-angle
limits come from the CHAMP robot yaml when you pass `robot:=go2|b2|xgo`.

Do **not** run `./run_teleop.sh` / `teleop_twist_keyboard` at the same time
(both publish `/cmd_vel`).

## Bluetooth (Model 1708)

The 1708 has a small sync button on the top edge next to the USB port.
Hold it until the Xbox button blinks, then:

```bash
sudo bluetoothctl
power on
agent on
default-agent
scan on
# wait for "Xbox Wireless Controller"
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
quit
```

`bluetoothctl info` `Connected: yes` is not enough. Stock `xpad` is USB.
Model 1708 over Bluetooth is HID (`Modalias: usb:v045Ep02FD...`) until the
kernel creates an input node. `/dev/input/js0` is **not** proof of an Xbox:
it is often `mouce-library-fake-mouse` or RustDesk. The Xbox must appear in
`/proc/bus/input/devices` with `Handlers=... jsN`.

```bash
grep -A25 -iE 'xbox|microsoft|045e|rustdesk|mouce' /proc/bus/input/devices
ls -l /dev/input/js* /dev/input/by-id
```

If Bluetooth is connected but there is **no Xbox input device at all**:

```bash
sudo modprobe hidp
echo 1 | sudo tee /sys/module/bluetooth/parameters/disable_ertm
# persist: echo 'options bluetooth disable_ertm=1' | sudo tee /etc/modprobe.d/xbox-bt.conf
bluetoothctl disconnect AA:BB:CC:DD:EE:FF
# power-cycle the pad, then:
bluetoothctl connect AA:BB:CC:DD:EE:FF
```

If `dmesg` shows `microsoft 0005:045E:02FD` **parse failed** / **error -22**
(`unknown main item tag 0x0`, `unbalanced collection`), stock
`hid-microsoft` cannot read the 1708 Bluetooth HID descriptor. Bluetooth
stays connected but **no input node is created**. Install
[xpadneo](https://github.com/atar-axis/xpadneo) (not optional on this pad):

```bash
sudo apt install dkms git linux-headers-$(uname -r)
git clone https://github.com/atar-axis/xpadneo.git
cd xpadneo && sudo ./install.sh
sudo reboot
```

After reboot, connect the pad, then `dmesg` should mention `hid-xpadneo`
(not `microsoft: probe ... failed`). Confirm:

```bash
lsmod | grep xpadneo
grep -A8 -i xbox /proc/bus/input/devices    # Handlers must include jsN
source /opt/ros/humble/setup.bash
ros2 run joy joy_enumerate_devices          # need a line named Xbox
```

USB cable into the 1708 also works with stock `xpad` if you need a pad before
xpadneo is installed.

```bash
jstest /dev/input/js1    # sudo apt install joystick; pick the jsN that is Xbox, not fake-mouse
```

You need to be in the `input` group: `sudo gpasswd -a $USER input` then log out
and back in.

```bash
sudo apt install ros-humble-joy    # required. Jazzy: ros-jazzy-joy
```

## Run

Sim or Sim2Real already running in another terminal, then from the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select xbox_one_s_teleop
source install/setup.bash
./run_xbox_teleop.sh go2          # or b2 / xgo
# same as: ros2 launch xbox_one_s_teleop teleop.launch.py robot:=go2
```

Confirm the pad: the launch log must say `Opened joystick: Xbox ...`, not
RustDesk. Then `ros2 topic echo /joy` (axes move when you move the sticks).

SDL index is **not** `/dev/input/jsN`. List what `joy_node` can see:

```bash
ros2 run joy joy_enumerate_devices
```

Default `device_id:=auto` picks the first name containing `Xbox` and skips
virtual pads (RustDesk). After xpadneo, SDL often lists the 1708 as
**Xbox 360 Controller** (PID spoof `0x028E`) twice, then RustDesk. That 360
name **is** the 1708 — do not pass `device_name:="Xbox Wireless Controller"`
(SDL will not match). `Mapped: false` is normal; if the right stick is axes
3/4 on `/joy`, launch `driver:=linux`.

```bash
./run_xbox_teleop.sh b2
./run_xbox_teleop.sh b2 device_id:=0
./run_xbox_teleop.sh b2 driver:=linux
```

If enumerate shows **only** `RustDesk UInput Keyboard`, the Xbox is not a
joystick yet. `js0` is often that RustDesk pad. Pair the 1708 until a second
line named `Xbox ...` appears (xpadneo if Bluetooth connects but no joystick),
then relaunch. Closing RustDesk can help.

If `/joy` right-stick axes are **3 and 4** (kernel `jstest` order, not SDL),
launch with `driver:=linux`:

```bash
./run_xbox_teleop.sh go2 driver:=linux
```

## Buttons (XInput / Linux js)

| Control | Action |
|---|---|
| **LB** | toggle **locomotion** ↔ **body pose** |
| **RB** (**hold**) | locomotion: enable `/cmd_vel` (**release = stop**) |
| **A** | `/body_pose` identity — body level, same as `{orientation: {w: 1.0}}` |
| **B** | zero `/cmd_vel` now |
| Left stick X + Y | locomotion: **vy** and **vx** together |
| Right stick X | locomotion: **yaw**; body-pose: **roll** |
| Right stick Y | body-pose: **pitch** (up = nose up) |

Mode is published on `/xbox_teleop/mode` (`locomotion` or `body_pose`).

Natural stick signs (ROS `+y` = left, `+yaw` = CCW, `+pitch` = nose **down**):

- stick up → walk forward / nose up
- stick right → strafe right / turn right / roll right-side-down

### Locomotion (default)

Hold **RB**, then left stick walks (forward/back **and** lateral at once) while
right stick X yaws. Releasing RB publishes zero Twist.

### Body pose

Press **LB** once. `/cmd_vel` is held at zero. Right stick X/Y are **rates**,
not a one-shot pose: full stick ≈ that robot's `body_pose.desired_rate` rad/s
(Go2 0.5 rad/s). Releasing the stick **holds** the last roll/pitch so you can
press **LB** again and walk while tilted.

The CLI example that pitches the body nose-up by 0.10 rad:

```bash
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose \
  "{orientation: {y: -0.04998, w: 0.99875}}"
```

is the same quaternion as RPY `(0, -0.10, 0)`. On the pad: body-pose mode,
hold the right stick **up** for about `0.10 / desired_rate` seconds (Go2 ≈ 0.2 s),
then release. Limits are `max_roll` / `max_pitch` in
`config/<robot>_body_pose.yaml` (Go2 0.25 / 0.20 rad).

**A** publishes identity, same as:

```bash
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose \
  "{orientation: {w: 1.0}}"
```

## If the robot does not walk

1. Look at the launch line `Opened joystick:`. If it is `RustDesk UInput Keyboard`
   (or anything that is not Xbox), it is the wrong SDL device. Run
   `ros2 run joy joy_enumerate_devices` and pass that Xbox `device_id:=`.
2. Hold **RB** (dead-man). Sticks do nothing without it.
3. Check `/xbox_teleop/mode` is `locomotion` (press LB if you are in body pose).
4. `ros2 topic echo /cmd_vel` while holding RB and pushing the left stick.
5. `ros2 topic echo /joy` — if right stick is axes 3/4, use `driver:=linux`.
