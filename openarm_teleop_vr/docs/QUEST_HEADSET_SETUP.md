# Driving OpenArm teleop from a Meta Quest 3s (real headset)

This is the end-to-end procedure to replace `fake_udp_sender.py` with a real
Quest 3s, using the vendor's prebuilt OpenXR app
(`openarmx-vr-quest.apk` from `github.com/openarmx/openarmx_teleop_vr_apk`,
branch `6.0_basic`).

## What the APK actually is (verified by inspecting the binary)

* Native **OpenXR** Android app (`XrRobotTeleopApp` in `libxrcontrollers.so`) —
  **not** Unity, no SteamVR/ALVR/PC-VR needed. It runs standalone on the Quest.
* Streams **UDP to `PC_IP:5100`** — the exact port and protocol the
  `openarm_teleop_bridge_vr` node already listens on.
* Per-frame packet (one per hand), space-separated text:
  ```
  <LEFT|RIGHT> px py pz qx qy qz qw trigger grip a b x y rate ts ...
  ```
  This matches the bridge's `parseHandPayload` exactly (extra trailing fields
  are ignored).
* This `6.0_basic` build is **relative-mode only** (no absolute/head/calibrate
  packets) — which is precisely what our MVP teleop implements. 
* In-app default target IP is `10.181.252.1`; there is an **“EDIT IP ADDRESS”**
  menu (open with the controller **menu** button) to point it at your PC.
* **Grip** engages tracking for that hand; **trigger** drives the gripper.

## This PC's facts (for substitution below)

* PC has two NICs — pick whichever network the Quest can actually join:
  * **`192.168.0.214`** (interface `enp1s0`, Aquantia) — plain LAN behind
    `192.168.0.1`. **Preferred**: no client isolation to worry about.
  * `10.149.50.185` (interface `enp0s31f6`) — campus/corporate `10.149.x.x/16`
    network; may have Wi-Fi client isolation that blocks UDP between devices.
  * Re-check either with `hostname -I` / `ip -brief addr show`.
* UDP **5100** is free. The Quest must join the **same subnet** as whichever IP
  you use below (i.e. connect to the Wi-Fi/AP behind `192.168.0.1` for the
  preferred option).

---

## Step 1 — Enable Developer Mode on the Quest 3s

Developer mode is required to sideload (`adb install`).

1. On a phone, install the **Meta Horizon** app and sign in with the **same Meta
   account** as the headset.
2. Create a developer org if you don't have one: visit
   `https://developers.meta.com/`, accept the developer agreement (this unlocks
   the Developer Mode toggle).
3. In the Meta Horizon phone app: **Menu → Devices →** select your Quest 3s **→
   Headset settings → Developer Mode →** toggle **ON**.
4. Reboot the headset (hold power, Restart).

## Step 2 — Install `adb` on the PC (no sudo needed)

This box has conda but no `adb`. Easiest sudo-free install:

```bash
conda install -y -c conda-forge android-platform-tools
# verify
adb version
```
(Alternative without conda: download Google "platform-tools" zip, unzip, and use
`./adb` from that folder.)

## Step 3 — Sideload the APK

1. Plug the Quest 3s into the PC with a USB-C cable.
2. Put the headset on — accept **“Allow USB debugging?”** (check *Always allow*).
3. From the PC:
   ```bash
   adb devices                       # should list your headset as 'device'
   cd ~/ROS_WS/src                    # or wherever you saved the apk
   # grab the apk if you don't have it:
   curl -sL -o openarmx-vr-quest.apk \
     https://raw.githubusercontent.com/openarmx/openarmx_teleop_vr_apk/6.0_basic/openarmx-vr-quest.apk
   adb install -r openarmx-vr-quest.apk
   ```
   `Success` means it's installed. If `adb devices` shows `unauthorized`, re-check
   the in-headset prompt.

## Step 4 — Network + firewall

* Put the Quest 3s on the **same subnet** as the PC's `192.168.0.214` interface —
  join whatever Wi-Fi/AP is behind `192.168.0.1`. (Falling back to the campus
  `10.149.x.x/16` network works too, but is more likely to hit Wi-Fi client
  isolation that silently drops the UDP packets.)
* The PC must accept inbound UDP 5100. If `ufw` is active:
  ```bash
  sudo ufw allow 5100/udp     # only if a firewall is enabled
  ```
* Sanity-check the PC is reachable from the Quest's network (from another device
  on that network: `ping 192.168.0.214`).

## Step 5 — Launch the app and point it at the PC

1. In the headset: **App Library → Unknown Sources →** launch the OpenArmX VR app.
2. Press the **menu** button on a controller to open **“EDIT IP ADDRESS”**.
3. Enter the PC IP **`192.168.0.214`** (port is fixed at 5100). Confirm — you
   should see a “UDP connection established to 192.168.0.214:5100” style state.

## Step 6 — Start the PC-side stack

Three shells (see the main README for detail):

```bash
# Shell 1 — Isaac Sim backend (Isaac's own python; no ROS sourcing)
cd ~/IsaacLab
~/env_isaaclab/bin/python ~/ROS_WS/src/openarm_teleop_vr/isaac/openarm_isaac_teleop.py

# Shell 2 — bridge + teleop + relay
cd ~/ROS_WS && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch openarm_teleop_vr teleop_vr.launch.py backend:=isaac
```
(No `vr_steamvr_udp_sender.py` and no fake sender — the headset is the source now.)

## Step 7 — Verify the headset data is arriving

```bash
# raw packets hitting the bridge port (quickest check):
# you should see text lines when you move/grip the controllers
sudo tcpdump -A -n udp port 5100        # or, no sudo:
python3 - <<'PY'
import socket
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.bind(('0.0.0.0',5101))
print("(point the app at port 5101 temporarily to peek, or use the topics below)")
PY

# the proper check — bridge output topics:
ros2 topic echo /vr_right_controller/pose      # moves as you move the controller
ros2 topic echo /vr_right_controller/grip      # ~1.0 when you squeeze grip
ros2 topic echo /right_forward_position_controller/commands   # teleop IK output
```
If poses appear but commands don't, confirm `/joint_states` is flowing from Isaac
(the relay republishes it) and that you're **holding grip** to engage.

## Step 8 — Operate

* **Hold grip** on a controller → that arm engages and the end-effector follows
  the controller's relative motion. **Release** → the arm holds position.
* **Trigger** → open/close that gripper.
* Both hands work independently/simultaneously.

## Step 9 — Tune the controller→robot mapping

Almost always needed on first run: real controller axes won't line up with the
robot base axes. Edit `config/teleop_params.yaml`:

* `position_scale_xyz` — per-axis translation gain (lower if motion feels too
  fast/large).
* `axis_matrix` — row-major 3×3 rotation mapping controller frame → robot base
  frame. If pushing the controller *forward* moves the EE *sideways*, replace
  identity with the right signed-permutation matrix. Practical method: engage
  grip, move the controller along **one** axis at a time, watch which robot axis
  responds in Isaac, and build the matrix so they agree.

Re-launch shell 2 after edits (params load at node start).

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `adb devices` shows `unauthorized` | Accept the USB-debugging prompt in-headset (Always allow) |
| `adb devices` empty | Bad/charge-only USB cable; try another cable/port |
| No packets on UDP 5100 | Quest not on same/routable network; client isolation on Wi-Fi; firewall; wrong IP in app |
| Poses arrive but arm doesn't move | Not holding grip; or `/joint_states` not flowing (Isaac/relay not running) |
| Arm moves on the wrong axis | Tune `axis_matrix` (Step 9) |
| Arm jitters / lunges | Lower `position_scale_xyz`; keep `max_step_deg_per_joint` small |
| App can't be found in headset | App Library → filter **Unknown Sources** |

## Switching this exact setup to the real robot later

Identical headset steps. Only the PC backend changes: instead of the Isaac script
+ relay, bring up OpenArm `ros2_control` with `use_fake_hardware:=false` and two
`forward_position_controller`s on the same command topics, and launch with
`backend:=real`. The headset, bridge, and teleop node are unchanged.
