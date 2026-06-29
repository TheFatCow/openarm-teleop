# openarm_teleop_vr

VR teleoperation for the **OpenArm v1.0 bimanual** robot, adapted from the
OpenArmX VR pipeline. Drive the robot from a **Meta Quest 3s** in **Isaac Sim**
now, and switch to the **real OpenArm** later without touching the teleop logic.

The closed vendor IK core from the original (`openarmx_arm_driver`) is replaced
by an **open Pinocchio-based core** (`teleop_core.py`).

## Architecture

```
Quest 3s ─(ALVR/SteamVR, Wi-Fi)─▶ PC SteamVR ─▶ vr_steamvr_udp_sender.py ─UDP─┐
                                                                              ▼
                       openarm_teleop_bridge_vr (C++)  UDP text ─▶ ROS 2 topics
                                                                              │
                       openarm_teleop_vr_node (Pinocchio IK, relative mode)   │
                          publishes Float64MultiArray [7 joints + gripper]/arm│
                              /left_forward_position_controller/commands  ─────┤
                              /right_forward_position_controller/commands      │
                                                                              ▼
   backend=isaac:  openarm_isaac_relay ─UDP/JSON─▶ openarm_isaac_teleop.py (Isaac Sim)
                                       ◀─UDP/JSON─ /joint_states
   backend=real :  OpenArm ros2_control + forward_position_controllers (CAN)
```

The teleop node is **backend-agnostic** — it only speaks the two command topics
and reads `/joint_states`. Sim and real differ only in *who provides* those.

### Why the Isaac relay?

Isaac Sim ships **Python 3.11**; ROS 2 Humble's `rclpy` is built for **3.10**, so
rclpy cannot be imported inside the Isaac process. `openarm_isaac_relay` (ROS,
3.10) therefore bridges the ROS topics to the Isaac process over a localhost
UDP/JSON link; `openarm_isaac_teleop.py` stays pure-Python (no ROS import).

## Install

```bash
# Pinocchio for the IK core (userspace; no sudo). Or: sudo apt install ros-humble-pinocchio
pip install --user pin

# Build the two packages (this workspace already has openarm_description).
cd ~/ROS_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_teleop_bridge_vr openarm_teleop_vr
source install/setup.bash
```

Isaac Sim 4.5 + IsaacLab are assumed installed (the OpenArm asset config lives at
`IsaacLab/source/isaaclab_assets/.../robots/openarm.py`).

## Run in simulation

Open **three** shells (all with ROS + the workspace overlay sourced, except the
Isaac one).

**1 — Isaac Sim backend** (no ROS sourcing; uses Isaac's own Python):
```bash
cd ~/IsaacLab
~/env_isaaclab/bin/python \
  ~/ROS_WS/src/openarm_teleop_vr/isaac/openarm_isaac_teleop.py    # add --headless to hide GUI
```
It spawns the OpenArm bimanual robot, listens for joint commands on UDP 6001 and
sends `/joint_states` data to UDP 6002.

**2 — Bridge + teleop + relay**:
```bash
ros2 launch openarm_teleop_vr teleop_vr.launch.py backend:=isaac
```
(`backend:=isaac` also starts `openarm_isaac_relay`.)

**3 — VR input** (pick one):
```bash
# A) No headset — scripted motion to prove the pipeline:
python3 ~/ROS_WS/src/openarm_teleop_vr/vr/fake_udp_sender.py --hands both

# B) Quest 3s via SteamVR (see "VR input" below):
pip install --user openvr
python3 ~/ROS_WS/src/openarm_teleop_vr/vr/vr_steamvr_udp_sender.py
```

Hold a controller's **grip** to engage that arm; move the controller and the
end-effector follows. The **trigger** opens/closes the gripper. Release grip to
hold position.

### Quick checks
```bash
ros2 topic echo /vr_right_controller/pose                         # bridge input
ros2 topic echo /right_forward_position_controller/commands       # teleop output
ros2 topic echo /joint_states                                     # backend feedback
```

## VR input (Quest 3s)

Building/sideloading an on-headset APK is out of scope here; the supported path
keeps everything on the PC:

1. Install **SteamVR** (Steam) and **ALVR** on the PC; sideload the **ALVR client**
   on the Quest 3s (e.g. via SideQuest). Pair so the Quest appears in SteamVR with
   two controllers.
2. `pip install --user openvr`, start SteamVR, then run `vr_steamvr_udp_sender.py`.

It reads both controllers' pose/trigger/grip from OpenVR and emits the bridge's
UDP text protocol. The axis index mapping (trigger/grip) may need tweaking per
controller — see the comments in that file.

*(Alternative for later: an on-headset OpenXR/Unity sender, or the original
`openarmx_teleop_vr_apk`, can replace the PC sender as long as it emits the same
UDP protocol to port 5100.)*

## Tuning the mapping

Relative mode anchors the controller and EE poses when you grip, then applies the
controller delta to the EE. Two parameters in `config/teleop_params.yaml` shape it:

* `position_scale_xyz` — per-axis gain on controller translation.
* `axis_matrix` — row-major 3×3 rotation from the controller frame to the robot
  base frame. Start at identity; if moving the controller right/up/forward maps to
  the wrong robot axis, set this to the appropriate signed-permutation matrix.

`max_step_deg_per_joint` caps per-tick joint motion (safety speed limit).

## Switching to the real OpenArm

The real OpenArm uses its own `ros2_control` stack (`openarm_description` ships the
xacro with the `openarm_hardware/OpenArmHW` CAN backend). To switch:

1. Bring up OpenArm `ros2_control` with `use_fake_hardware:=false` and two
   `forward_position_controller`s publishing to
   `/{left,right}_forward_position_controller/commands` (7 arm joints + gripper).
2. Run the bridge + teleop node exactly as in sim, with `backend:=real`
   (no Isaac, no relay).

No change to `teleop_core.py` or the teleop node is required.

> ⚠️ Start the robot/backend **before** the teleop node, and keep
> `max_step_deg_per_joint` small on first runs.

## Files

| Path | Role |
|------|------|
| `openarm_teleop_vr/teleop_core.py` | Open Pinocchio IK + relative teleop (no ROS) |
| `openarm_teleop_vr/openarm_teleop_vr_node.py` | ROS adapter (subscribe VR + joint_states → commands) |
| `openarm_teleop_vr/isaac_relay_node.py` | ROS ↔ Isaac UDP/JSON relay |
| `isaac/openarm_isaac_teleop.py` | Isaac Sim backend (ROS-free) |
| `vr/vr_steamvr_udp_sender.py` | Quest→SteamVR→UDP sender |
| `vr/fake_udp_sender.py` | Headset-free test source |
| `test/test_teleop_core.py` | Offline IK / teleop unit tests |
| `../openarm_teleop_bridge_vr/` | C++ UDP→ROS bridge |

## Tests

```bash
python3 ~/ROS_WS/src/openarm_teleop_vr/test/test_teleop_core.py   # IK round-trip, relative motion
```
