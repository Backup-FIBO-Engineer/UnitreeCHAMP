"""Xbox One S Model 1708 (Bluetooth) teleop for this CHAMP workspace.

Pairs with `joy_node` (`ros-humble-joy`). Stick mapping is
`config/xbox_one_s_1708_bt.yaml`. Speed and body-angle limits come from the
CHAMP robot yaml when you pass `robot:=go2|b2|xgo`.

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

On Ubuntu the stock `xpad` driver is often enough. If the pad connects but
`/dev/input/js0` never appears, install [xpadneo](https://github.com/atar-axis/xpadneo)
(the usual fix for 1708 over Bluetooth).

```bash
ls /dev/input/js*
jstest /dev/input/js0    # sudo apt install joystick
```

You need to be in the `input` group: `sudo gpasswd -a $USER input` then log out.

```bash
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy   # joy is required
```

Jazzy: `ros-jazzy-joy`.

## Run

Sim or Sim2Real already running in another terminal, then:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xbox_one_s_teleop teleop.launch.py robot:=go2
# or: ./run_xbox_teleop.sh go2
```

Do not run `teleop_twist_keyboard` at the same time (both publish `/cmd_vel`).

## Buttons (XInput / Linux js)

| Control | Action |
|---|---|
| **LB** | toggle **locomotion** ↔ **body pose** |
| **RB** (hold) | locomotion: enable `/cmd_vel` (release = stop) |
| **A** | `/body_pose` identity — body level, same as `orientation: {w: 1.0}` |
| **B** | zero `/cmd_vel` now |
| Left stick X/Y | locomotion: **vy** / **vx** together |
| Right stick X | locomotion: **yaw**; body-pose: **roll** |
| Right stick Y | body-pose: **pitch** (up = nose up) |

Mode is published on `/xbox_teleop/mode` (`locomotion` or `body_pose`).

In body-pose mode `/cmd_vel` is held at zero. Stick deflection is a **rate**
(full stick ≈ `desired_rate` rad/s of that robot). Releasing the stick **holds**
the last roll/pitch so you can switch back to locomotion and walk while tilted.
Limits are `max_roll` / `max_pitch` of `config/<robot>_body_pose.yaml`.

If `/joy` right-stick axes are 3 and 4 (kernel `jstest` order, not SDL), launch
`driver:=linux` to load `xbox_one_s_1708_bt_linuxjs.yaml`. Confirm with
`ros2 topic echo /joy`.
