# Xbox One S Model 1708 (Bluetooth) teleop

ROS 2 Humble package (`xbox_one_s_teleop`) for this workspace: `/cmd_vel` for
`unitree_rl_deploy`. Pair with `joy_node` (`ros-humble-joy`). Stick indices
are `config/xbox_one_s_1708_bt.yaml` (SDL2). Speed limits come from
`config/<robot>_rl.yaml` `max_cmd` when you pass `robot:=go2|b2`.

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
./run_xbox_teleop.sh go2          # or b2
# same as: ros2 launch xbox_one_s_teleop teleop.launch.py robot:=go2
```

Confirm the pad: the launch log must say `Opened joystick: Xbox ...`, not
RustDesk. Then `ros2 topic echo /joy` (axes move when you move the sticks).

SDL index is **not** `/dev/input/jsN`. List what `joy_node` can see:

```bash
ros2 run joy joy_enumerate_devices
```

After xpadneo, SDL lists the 1708 as **Xbox 360 Controller**. `driver:=auto`
(the default) then loads the Linux axis map (right stick = axes 3/4) and the
stick signs for that path: up = forward, right-stick left = turn left.
Hold **RB** to walk.

```bash
./run_xbox_teleop.sh b2
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
| **RB** (**hold**) | enable `/cmd_vel` (**release = stop**) |
| **B** | zero `/cmd_vel` now |
| Left stick X + Y | **vy** and **vx** together |
| Right stick X | **yaw** |

Natural stick signs (ROS `+y` = left, `+yaw` = CCW):

- stick up → walk forward
- stick right → strafe right / turn right

Hold **RB**, then left stick walks (forward/back **and** lateral at once) while
right stick X yaws. Releasing RB publishes zero Twist.

## If the robot does not walk

1. Look at the launch line `Opened joystick:`. If it is `RustDesk UInput Keyboard`
   (or anything that is not Xbox), it is the wrong SDL device. Run
   `ros2 run joy joy_enumerate_devices` and pass that Xbox `device_id:=`.
2. Hold **RB** (dead-man). Sticks do nothing without it.
3. `ros2 topic echo /cmd_vel` while holding RB and pushing the left stick **forward**.
   `+vx` is walk forward. The node logs `invert_vx` at startup and `/cmd_vel`
   once a second while walking.
4. If that echo is **negative** while the stick is forward, the mapping yaml
   sign does not match this `joy_node` path. Do not edit the yaml first — overlay:
   `./run_xbox_teleop.sh b2 invert_vx:=true` (or `invert_vx:=false` if the yaml
   already inverts). Same for `invert_vy` / `invert_yaw`.
5. If `/cmd_vel` is already **positive** and the robot still only walks backward,
   the policy observation or `default_angles` likely do not match training.
6. `ros2 topic echo /joy` — if right stick is axes 3/4, use `driver:=linux`.
