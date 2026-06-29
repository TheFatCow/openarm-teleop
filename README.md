# OpenArm VR Teleoperation

VR teleoperation for the **OpenArm v1.0 bimanual** robot, driven from a
**Meta Quest 3s**, running in **Isaac Sim** today and designed to switch to the
**real OpenArm** with no change to the teleop logic.

This is an open re-implementation of the OpenArmX VR pipeline for the open
[OpenArm](https://github.com/enactic/openarm) hardware. The original vendor code
shipped a **closed IK binary** (`openarmx_arm_driver.PinocchioTeleopCore`) that
isn't distributed; here that core is replaced by an **open Pinocchio-based IK +
relative-teleop core** you can read, test, and modify.

## Repository layout

| Folder | What it is |
|--------|------------|
| `openarm_teleop_vr/` | **Python ROS 2 package** — open Pinocchio IK core, teleop node, Isaac relay, VR senders, launch/config, tests |
| `openarm_teleop_bridge_vr/` | **C++ ROS 2 package** — UDP → ROS 2 bridge for VR controller data |
| `openarmx_teleop_vr/` | Original OpenArmX vendor pipeline, kept for reference |

## Pipeline

```
Quest 3s ─(OpenXR app, UDP)─▶ openarm_teleop_bridge_vr ─▶ ROS 2 topics
                                                              │
        openarm_teleop_vr_node (Pinocchio IK, relative mode) │
            publishes Float64MultiArray [7 joints + gripper]/arm
              /left_forward_position_controller/commands  ────┤
              /right_forward_position_controller/commands     │
                                                              ▼
  backend=isaac:  openarm_isaac_relay ⇄(UDP/JSON)⇄ Isaac Sim (openarm_isaac_teleop.py)
  backend=real :  OpenArm ros2_control + forward_position_controllers (CAN)
```

The teleop node is **backend-agnostic** — it only speaks the two command topics
and reads `/joint_states`. Sim vs. real differ only in *who provides* those.

> **Why a relay for Isaac?** Isaac Sim ships Python 3.11; ROS 2 Humble's `rclpy`
> is built for 3.10 and can't be imported in the Isaac process. `openarm_isaac_relay`
> (ROS, 3.10) bridges the ROS topics to the Isaac process over localhost UDP/JSON,
> keeping the Isaac script ROS-free.

## Quick start (simulation)

```bash
# 1) Build (in a colcon workspace; e.g. symlink these packages into ~/ROS_WS/src)
cd ~/ROS_WS && source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_teleop_bridge_vr openarm_teleop_vr
source install/setup.bash

# 2) Isaac Sim backend (Isaac's own python; no ROS sourcing)
~/env_isaaclab/bin/python ~/ROS_WS/src/openarm_teleop_vr/isaac/openarm_isaac_teleop.py

# 3) Bridge + teleop + relay
ros2 launch openarm_teleop_vr teleop_vr.launch.py backend:=isaac

# 4) VR input — no headset needed to test:
python3 ~/ROS_WS/src/openarm_teleop_vr/vr/fake_udp_sender.py --hands both
```

Hold a controller's **grip** to engage that arm; move it and the end-effector
follows. **Trigger** opens/closes the gripper. Release grip to hold.

## Documentation

- **`openarm_teleop_vr/README.md`** — full package reference, run instructions,
  mapping/tuning, and the sim → real switch.
- **`openarm_teleop_vr/docs/QUEST_HEADSET_SETUP.md`** — step-by-step guide to
  driving teleop from a real Meta Quest 3s (dev mode, sideloading the OpenXR app,
  network, in-app IP config, verification).
- **`openarm_teleop_bridge_vr/README.md`** — the UDP → ROS bridge and its topics.

## Tests

```bash
python3 openarm_teleop_vr/test/test_teleop_core.py   # offline IK round-trip + relative-motion
```

## Status

MVP verified end-to-end: fake VR → bridge → Pinocchio IK → relay → **real Isaac
Sim physics** → `/joint_states` → teleop closed loop (simulated arm tracked
controller motion). Real-headset path documented but not yet hardware-tested.
**Not yet implemented** (deferred): absolute/head-anchored mode, calibration,
button macros (home/hands-up), rate scaling.

## Dependencies

ROS 2 Humble · Pinocchio (`pip install --user pin`, or `ros-humble-pinocchio`) ·
Isaac Sim 4.5 + IsaacLab (for the sim backend) · `openarm_description` (URDF) ·
`openvr` (only for the SteamVR sender path).

## License & attribution

`openarm_teleop_bridge_vr/` and `openarmx_teleop_vr/` derive from the OpenArmX
project and are licensed **CC BY-NC-SA 4.0** (non-commercial, share-alike,
attribution — © Chengdu Changshu Robot Co., Ltd.). The new Python sources under
`openarm_teleop_vr/openarm_teleop_vr/`, `isaac/`, and `vr/` are licensed
**Apache-2.0**. See per-file headers and `LICENSE` files. Robot model © Enactic
(`openarm_description`).
