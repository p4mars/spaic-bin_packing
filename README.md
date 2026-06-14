# AE4ASM527 Group 3

Autonomous **A↔B marker-shuttle** robot built on the MIRTE Master platform
(ROS 2 Humble). Dropped into an **unknown arena**, the robot builds a map from
scratch, finds two ArUco-marked zones with its camera, and shuttles back and forth
between them — picking up a handle at **Zone A**, placing/stacking a box at
**Zone B** — while avoiding static **and** moving obstacles.

Covers the full pipeline: **live SLAM mapping → Nav2 navigation with obstacle
avoidance → ArUco zone detection → camera fine-alignment → handle grasp (Zone A) →
precise dock + box placement (Zone B)**.

Unlike a "map first, then drive in a saved map" workflow, this project maps **live
during the mission** — there is no separate mapping phase and no saved map file
(the arena is unknown at start).

> The project is split across **three packages by role**. This file is the
> project-level overview; two deeper dives live alongside it:
> **[README_MAP.md](README_MAP.md)** (mapping) and **[README_NAV.md](README_NAV.md)**
> (navigation).

---

## Table of Contents
1. [System Architecture](#1-system-architecture)
2. [Node Communication Diagram](#2-node-communication-diagram)
3. [The three packages (and who made what)](#3-the-three-packages-and-who-made-what)
4. [Prerequisites](#4-prerequisites)
5. [Build & Install](#5-build--install)
6. [Configuration](#6-configuration)
7. [Running the mission](#7-running-the-mission)
8. [Test / partial modes](#8-test--partial-modes)
9. [Topic & Service Contract](#9-topic--service-contract)
10. [Troubleshooting](#10-troubleshooting)
11. [File Reference](#11-file-reference)
12. [Quick-start Cheatsheet](#12-quick-start-cheatsheet)

---

## 1. System Architecture

```
╔══════════════════════════════════════════════════════════════════════════╗
║                    MIRTE A↔B MARKER-SHUTTLE SYSTEM                        ║
╠══════════════════════════════════════════════════════════════════════════╣
║                                                                          ║
║  ┌────────────────────────────────────────────────────────────────┐     ║
║  │                     SENSORS (robot hardware)                     │     ║
║  │  RPLidar          RealSense camera          Wheel encoders       │     ║
║  │  /scan            /camera/color/image_raw   /…/odom + odom→base  │     ║
║  │  (360° laser)     /camera/depth/image_raw   (TF)                 │     ║
║  │                   /camera/.../camera_info                        │     ║
║  └──────────┬───────────────────────┬─────────────────────┬────────┘     ║
║             │                       │                     │              ║
║  ┌──────────▼──────────┐  ┌─────────▼─────────┐  ┌────────▼──────────┐   ║
║  │  MAPPING (SLAM)     │  │  ZONE DETECTION   │  │  NAVIGATION (Nav2) │   ║
║  │                     │  │                   │  │                    │   ║
║  │  scan_filter        │  │  zone_detector    │  │  planner_server    │   ║
║  │  → /scan_filtered   │  │  (ArUco A=100,    │  │  (NavFn/Dijkstra)  │   ║
║  │                     │  │   B=101+102)      │  │  controller_server │   ║
║  │  slam_toolbox       │  │  → /zone_a_pose   │  │  (Reg. Pure        │   ║
║  │  → /map             │  │  → /zone_b_pose   │  │   Pursuit)         │   ║
║  │  → map→odom TF      │  │                   │  │  bt_navigator      │   ║
║  └──────────┬──────────┘  └─────────┬─────────┘  │  behavior_server   │   ║
║             │                       │            │  lifecycle_manager │   ║
║             │                       │            │  + 2 costmaps      │   ║
║             │                       │            └─────────┬──────────┘   ║
║             │                       │                      │              ║
║  ┌──────────▼───────────────────────▼──────────────────────▼─────────┐   ║
║  │                  MISSION BRAIN  —  shuttle_manager                 │   ║
║  │   FSM: WAIT_SLAM → SEARCH → SHUTTLE(A↔B × round_trips) → DONE      │   ║
║  │   • sends NavigateToPose goals to Nav2 (drive to stand-offs)       │   ║
║  │   • drives wheels directly for: search spin, camera align, 180°    │   ║
║  │   • sets per-leg inflation + cruise speed live                     │   ║
║  │   • hands off to the Zone A / Zone B sub-systems below             │   ║
║  └───────────────┬────────────────────────────────┬──────────────────┘   ║
║                  │ grasp_at_a                     │ dock_at_b            ║
║  ┌───────────────▼──────────────┐  ┌──────────────▼──────────────────┐   ║
║  │  ZONE A — HANDLE GRASP        │  │  ZONE B — DOCK + BOX PLACE       │   ║
║  │  (mirte_perception, laptop)   │  │  (mirte_placement, robot)        │   ║
║  │                               │  │                                  │   ║
║  │  perception_node (YOLOv8)     │  │  marker_navigator (ArUco dock)   │   ║
║  │  → /perception/object_markers │  │  → P-control to marker midpoint  │   ║
║  │                               │  │  → /robot_positioned, drive-back │   ║
║  │  grasp_node (visual servo+IK) │  │                                  │   ║
║  │  → service /grasp_handle      │  │  box_placer (arm joint poses)    │   ║
║  │  → drives base + arm + grip   │  │  → lower, release, stack, home   │   ║
║  └───────────────────────────────┘  └──────────────────────────────────┘   ║
║                                                                          ║
╚══════════════════════════════════════════════════════════════════════════╝
```

---

## 2. Node Communication Diagram

```
                 ┌───────────────────────────────────────────────┐
                 │             shuttle_manager (FSM)             │
                 │                                               │
                 │  Subscribes: /zone_a_pose  /zone_b_pose        │
                 │              /perception/object_markers        │
                 │              /box_placed  /robot_turned_around  │
                 │  Action client: navigate_to_pose (Nav2)        │
                 │  Service client: /grasp_handle                 │
                 │  Publishes: /…/cmd_vel (direct drive),         │
                 │             arm trajectory + gripper command   │
                 └───────┬───────────────────────┬───────────────┘
                         │ NavigateToPose         │ spawns (ros2 run) / calls service
          ┌──────────────▼─────────────┐         │
          │  Nav2 (one container)      │   ┌──────┴───────────────────────┐
          │  planner / controller /    │   │                              │
          │  bt_navigator / behaviors  │   ▼                              ▼
          │  / lifecycle / 2 costmaps  │  ZONE A (laptop)         ZONE B (robot)
          └──────────────┬─────────────┘  mirte_perception        mirte_placement
                         │ Twist          ┌────────────────┐      ┌──────────────────┐
                         ▼                │ perception_node│      │ marker_navigator │
        /mirte_base_controller/cmd_vel   │ → markers      │      │  ↕ box_placer    │
                         │                │ grasp_node     │      │ (handshake topics)│
                         ▼                │ → /grasp_handle│      └──────────────────┘
                       WHEELS             └────────────────┘

  Mapping feed:
    /scan ─► scan_filter ─► /scan_filtered ─► slam_toolbox ─► /map + map→odom TF
                                          └─► Nav2 costmaps (obstacle layer)

  TF tree:
    map → odom → base_link → laser
                           → camera_link → camera_depth_optical_frame
                           → (arm / gripper links)
    (map→odom: slam_toolbox · odom→base_link: base controller · mounts: robot_state_publisher)
```

**One full A→B cycle (with all stages enabled):**
```
  [shuttle_manager]  SEARCH → both zones found → start shuttle
  [Nav2]             drive to 1 m stand-off in front of Zone A (marker 100)
  [shuttle_manager]  camera fine-align exactly in front of the marker
  [mirte_perception] handle detected on /perception/object_markers
                     → /grasp_handle called → base+arm+gripper grasp the handle
  [shuttle_manager]  grasp returns → set carry inflation → drive to Zone B
  [Nav2]             drive to stand-off in front of Zone B (markers 101+102)
  [shuttle_manager]  camera fine-align at the midpoint
  [mirte_placement]  marker_navigator docks precisely → /robot_positioned
                     box_placer lowers + releases + stacks the box → /box_placed
                     marker_navigator turns 180° → /robot_turned_around
  [shuttle_manager]  advance leg → return to Zone A (smaller inflation) …
                     repeat for round_trips, then DONE
```

---

## 3. The three packages (and who made what)

| Package | Directory | Runs on | Role |
|---|---|---|---|
| **`mirte_driving`** | `src/mirte_driving` | robot | Mapping (SLAM feed) + Nav2 config + zone detection + the mission FSM that orchestrates everything |
| **`mirte_placement`** | `src/mirte_placement` | robot | **Zone B**: precise ArUco dock (`marker_navigator`) + box lay-down/stacking (`box_placer`) |
| **`mirte_perception`** | `src/mirte-ros-packages/mirte_perception` | laptop | **Zone A**: YOLOv8 handle detection (`perception_node`) + visual-servo grasp (`grasp_node`, service `/grasp_handle`) |

Provenance, at a glance (full breakdown in README_MAP §7 and README_NAV §2):

- **Custom project code:** `scan_filter`, `zone_detector`, `shuttle_manager`
  (in `mirte_driving`); `marker_navigator`, `box_placer` (in `mirte_placement`);
  `perception_node`, `grasp_node` (in `mirte_perception`); all launch files.
- **From the `mirte_navigation` reference (adapted):** the behaviour tree
  (`nav2_minimal_tree.xml`, shipped by the reference) and the overall SLAM/Nav2
  *approach*. The parameter files (`slam_params.yaml`, `exploration_nav2_params.yaml`)
  are project-authored, modelled on the reference's minimal configs and tuned for
  this robot. The `scan_filter` / `/scan_filtered` step is a project addition — the
  reference mapped from the raw `/scan`.
- **Installed / standard (apt):** `slam_toolbox` (SLAM engine), the Nav2 stack,
  `tf2_ros`, `pointcloud_to_laserscan`, YOLO (`ultralytics`), `ikpy`.
- **Robot vendor (preinstalled):** the base controller (`odom`, `odom→base_link`
  TF), `robot_state_publisher` (sensor-mount TFs), the lidar / camera / arm
  drivers.

---

## 4. Prerequisites

### On the robot (Ubuntu 22.04, ROS 2 Humble)
```bash
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-tf2-ros \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-teleop-twist-keyboard \
  python3-opencv \
  ros-humble-cv-bridge
```
`mirte_driving` needs `slam_toolbox` + Nav2; `zone_detector` and
`marker_navigator` need OpenCV (`cv2.aruco`).

### On the laptop (for Zone A grasping, `grasp_at_a`)
```bash
pip install ultralytics ikpy
```
`perception_node` runs YOLOv8 (`ultralytics`); `grasp_node` does inverse
kinematics with `ikpy`. These run on the laptop because YOLO is heavy for the
robot's onboard computer; the robot streams its camera and serves the arm.

### YOLO models (Zone A)
Trained weights live in `mirte_perception/models/`:
`handles_model.pt`, `boxes_model.pt`, `gripper_model.pt`. `grasp.launch.py`
defaults to that folder; override per-file with `model1_path:=…` etc. if the
weights are elsewhere.

---

## 5. Build & Install

```bash
cd ~/spatial-ai/ws

# Robot packages
colcon build --packages-select mirte_driving mirte_placement --symlink-install

# Laptop (Zone A)
colcon build --packages-select mirte_perception --symlink-install

source install/setup.bash
# add to ~/.bashrc so every shell is sourced:
echo "source ~/spatial-ai/ws/install/setup.bash" >> ~/.bashrc
```

---

## 6. Configuration

There are no saved-map / station-coordinate files to edit (the arena is unknown
and mapped live). Everything is set through **launch arguments / ROS parameters**.

### 6.1 ArUco markers
| Zone | Marker IDs | Topic produced | Dictionary / size |
|---|---|---|---|
| **A** (single marker) | `100` | `/zone_a_pose` | `DICT_4X4_250`, 8 cm |
| **B** (midpoint of a pair) | `101` + `102` | `/zone_b_pose` | `DICT_4X4_250`, 8 cm |

The same `101`/`102` pair is reused by `mirte_placement` for the precise dock.
Print from the `DICT_4X4_250` family; **the physical size must match
`zone_marker_size` / `dock_marker_size` (0.08 m)** — every camera distance scales
with it.

### 6.2 Key mission arguments (`mission.launch.py`)
| Arg | Default | Meaning |
|---|---|---|
| `round_trips` | `3` | number of A→B→A round trips |
| `arm_mimic` | `true` | canned arm carry/drop gestures (no real grasp/place) |
| `grasp_at_a` | `false` | engage `mirte_perception` handle grasp at A (laptop) |
| `dock_at_b` | `false` | engage `mirte_placement` precise dock + box place at B |
| `run_zone_detector` | `false` | detect zones on the robot (true) or offload to the laptop (false) |
| `use_depth_scan` | `false` | synthesise `/scan` from the depth camera (no-lidar units) |
| `scan_min_range` | `0.40` | self-return cutoff (raise if the arm sits in the lidar plane) |
| `udp_only` | `true` | UDP-only DDS profile (avoids a shared-memory clash) |

### 6.3 Tuning files
- `params/slam_params.yaml` — SLAM (resolution, scan-matching). See README_MAP §5.
- `params/exploration_nav2_params.yaml` — Nav2 (speeds, footprint, **inflation
  radius**, costmaps). See README_NAV §9.
- `trees/nav2_minimal_tree.xml` — Nav2 behaviour on success/failure.

---

## 7. Running the mission

The mission is a **single launch** — SLAM, Nav2, zone detection, and the mission
FSM all come up together and the map is built live.

```bash
# ── ROBOT ──
ros2 launch mirte_driving mission.launch.py
```
By default zone detection is **offloaded to the laptop**, so also start the
detector there (same network + same `ROS_DOMAIN_ID`):
```bash
# ── LAPTOP ──
ros2 launch mirte_driving detector.launch.py
```

**Full merged mission** (handle grasp at A + box place at B):
```bash
# ── ROBOT ──
ros2 launch mirte_driving mission.launch.py dock_at_b:=true grasp_at_a:=true
# ── LAPTOP ── zone detection
ros2 launch mirte_driving detector.launch.py
# ── LAPTOP ── Zone A grasp stack (YOLO + IK)
ros2 launch mirte_perception grasp.launch.py
```
`mirte_placement` (Zone B) is **spawned automatically** by `shuttle_manager` when
`dock_at_b:=true` — no separate launch needed.

**Expected log arc:**
```
SLAM/TF ready → Spinning to find zones (have […]) → Both zones found —
starting shuttle → Cruise speed → 0.32 → Leg 1/N → Zone A →
Aligned in front of Zone A → … → Leg 2 → Zone B → Aligned in front of Zone B →
turning 180° → return → … → DONE
```

**Emergency stop** (kill wheel commands):
```bash
ros2 topic pub /mirte_base_controller/cmd_vel geometry_msgs/msg/Twist "{}" --once
```

---

## 8. Test / partial modes

**Navigation-only** (pure A↔B shuttle, wheels + camera align, no arm/grasp/place):
```bash
# robot
ros2 launch mirte_driving mission.launch.py \
  dock_at_b:=false grasp_at_a:=false arm_mimic:=false round_trips:=2
# laptop
ros2 launch mirte_driving detector.launch.py
```

**Verify mapping alone** (drive by hand, watch the map build):
```bash
ros2 run tf2_ros tf2_echo map base_link        # localisation alive?
ros2 topic hz /map                             # map updating (~1 Hz)?
ros2 topic echo /scan_filtered --once --field range_min   # filter alive?
# manual drive:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/mirte_base_controller/cmd_vel
```

**Bench-test the Zone A grasp** (no mission):
```bash
ros2 launch mirte_perception grasp.launch.py          # laptop
ros2 topic echo /perception/object_markers            # hold a handle in view
ros2 service call /grasp_handle std_srvs/srv/Trigger  # run one grasp
```

**Bench-test the Zone B placement** (no mission):
```bash
ros2 run mirte_placement marker_navigator.py          # docks on markers 101/102
ros2 run mirte_placement box_placer.py                # auto-starts on /robot_positioned
```

---

## 9. Topic & Service Contract

### Mapping & navigation
| Name | Type | Direction | Notes |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | lidar → `scan_filter` | raw, BEST_EFFORT QoS |
| `/scan_filtered` | `sensor_msgs/LaserScan` | `scan_filter` → SLAM + costmaps | self-returns removed |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` → costmaps, FSM | live occupancy grid |
| `navigate_to_pose` | `nav2_msgs/NavigateToPose` (action) | `shuttle_manager` → Nav2 | every leg + wander hop |
| `/mirte_base_controller/cmd_vel` | `geometry_msgs/Twist` | Nav2 **or** FSM → base | exactly one driver at a time |

### Zone detection (zone_detector)
| Name | Type | Notes |
|---|---|---|
| `/camera/.../image_raw(/compressed)`, `/camera/camera_info` | `Image` / `CompressedImage` / `CameraInfo` | camera input |
| `/zone_a_pose`, `/zone_b_pose` | `geometry_msgs/PoseStamped` (`map` frame) | the goal poses |

### Zone A — handle grasp (mirte_perception)
| Name | Type | Direction | When |
|---|---|---|---|
| `/perception/object_markers` | `visualization_msgs/MarkerArray` | `perception_node` → FSM | handle detections (ns `model1_sphere`) |
| `/perception/annotated_image` | `sensor_msgs/Image` | `perception_node` → (debug) | YOLO overlay |
| `/grasp_handle` | `std_srvs/srv/Trigger` (service) | FSM → `grasp_node` | called once a handle is seen; blocks during the grasp |
| `/grasp/proceed` | `std_msgs/Bool` | `grasp_node` → (status) | grasp progress |

### Zone B — dock + place (mirte_placement handshake)
| Name | Type | Direction | Meaning |
|---|---|---|---|
| `/aruco_101_pose`, `/aruco_102_pose` | `PoseStamped` | `marker_navigator` → (debug) | live marker poses |
| `/robot_positioned` | `Bool` (latched) | `marker_navigator` → `box_placer` | docked → start placing |
| `/arm_placed` | `Bool` (latched) | `box_placer` → `marker_navigator` | arm lowered → drive back |
| `/robot_backed_up` | `Bool` | `marker_navigator` → `box_placer` | backed up → open gripper |
| `/box_placed` | `String` | `box_placer` → `marker_navigator` + FSM | box released, sequence done |
| `/robot_turned_around` | `Bool` | `marker_navigator` → FSM | 180° done → leg complete |
| `/navigation_failed`, `/place_failed` | `Bool` (latched) | → (failsafe) | timeout aborts |
| `/start_placing`, `/reset_stack` | `Bool` | (manual) → `box_placer` | manual trigger / zero stack |

### Arm & gripper (vendor controllers, used by FSM + box_placer + grasp_node)
| Name | Type |
|---|---|
| `/mirte_master_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| `/mirte_master_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |

---

## 10. Troubleshooting

**Robot never starts navigating / "Waiting for SLAM (map→base_link)…"**
```bash
ros2 run tf2_ros tf2_echo map base_link    # map→base_link must resolve
ros2 topic hz /scan                        # lidar alive?
ros2 run tf2_ros tf2_echo odom base_link   # base publishing odom→base_link?
```
No `/scan` → lidar / `scan_filter`. No `odom→base_link` → the robot's base driver
isn't up. (See README_MAP §8.)

**Map smears / rotated copies of the room** → the search/turn spins are too fast
for scan-matching; keep spins slow (already ≤0.4 rad/s). (README_MAP §6.)

**Robot frozen, "collision ahead" everywhere** → the arm/chassis is inside the
lidar plane; raise `scan_min_range` (e.g. `0.45`).

**Spins forever, `have ['A']`** → both Zone B markers (101 *and* 102) aren't in
view; reposition, and check the marker IDs / dictionary / printed size.

**`failed to create plan` between zones** → carry inflation too wide for a tight
gap; lower `inflation_carry`. (README_NAV §7.)

**Distance to a zone consistently wrong** → `zone_marker_size` ≠ the printed tag
size; all camera distances scale with it.

**Zone A grasp does nothing** → check the laptop side:
```bash
ros2 topic hz /camera/color/image_raw      # robot streaming the camera?
ros2 topic echo /perception/object_markers # YOLO seeing the handle?
python3 -c "import ultralytics, ikpy"      # deps installed?
```

**Zone B place stalls** → the handshake is waiting on a signal:
```bash
ros2 topic echo /robot_positioned   # did marker_navigator finish docking?
ros2 topic echo /arm_placed         # did box_placer lower the arm?
ros2 node list | grep -E "marker_navigator|box_placer"
```

**DDS / shared-memory errors at start-up** → launch with `udp_only:=true`
(already the mission default).

---

## 11. File Reference

```
src/mirte_driving/                      package name: mirte_driving   (robot)
├── mirte_driving/
│   ├── scan_filter.py        lidar self-return filter → /scan_filtered     [MAPPING]
│   ├── zone_detector.py      ArUco A/B detection → /zone_a_pose,/zone_b_pose [NAV]
│   └── shuttle_manager.py    mission FSM: search, shuttle, align, hand-offs [NAV]
├── params/
│   ├── slam_params.yaml             slam_toolbox config                    [MAPPING]
│   └── exploration_nav2_params.yaml Nav2 + costmaps + inflation            [NAV]
├── trees/nav2_minimal_tree.xml      Nav2 behaviour tree                    [NAV]
├── config/fastdds_udp_only.xml      UDP-only DDS profile
├── launch/
│   ├── mission.launch.py     full mission (SLAM+Nav2+zones+FSM) — ROBOT
│   ├── shuttle.launch.py     the stack mission.launch wraps (sim defaults)
│   └── detector.launch.py    zone_detector only — LAPTOP
├── README.md                 ← you are here (project overview)
├── README_MAP.md             mapping deep-dive
└── README_NAV.md             navigation deep-dive

src/mirte_placement/                    package: mirte_placement        (robot, Zone B)
└── mirte_placement/
    ├── marker_navigator.py   precise ArUco dock + drive-back + 180° turn
    └── box_placer.py         arm lower / release / stack / return-home

src/mirte-ros-packages/mirte_perception/  package: mirte_perception     (laptop, Zone A)
├── mirte_perception/
│   ├── perception_node.py    YOLOv8 handle/box detection → object_markers
│   └── grasp_node.py         visual-servo + IK grasp; service /grasp_handle
├── models/                   handles_model.pt · boxes_model.pt · gripper_model.pt
└── launch/grasp.launch.py    starts perception_node + grasp_node
```

---

## 12. Quick-start Cheatsheet

```bash
# ── Build ──
cd ~/spatial-ai/ws
colcon build --packages-select mirte_driving mirte_placement --symlink-install
source install/setup.bash

# ── Navigation-only shuttle (robot + laptop detector) ──
ros2 launch mirte_driving mission.launch.py arm_mimic:=false round_trips:=2   # robot
ros2 launch mirte_driving detector.launch.py                                  # laptop

# ── Full mission: grasp at A + place at B ──
ros2 launch mirte_driving mission.launch.py dock_at_b:=true grasp_at_a:=true  # robot
ros2 launch mirte_driving detector.launch.py                                  # laptop
ros2 launch mirte_perception grasp.launch.py                                    # laptop

# ── Emergency stop ──
ros2 topic pub /mirte_base_controller/cmd_vel geometry_msgs/msg/Twist "{}" --once
```

Deeper detail: **[README_MAP.md](README_MAP.md)** · **[README_NAV.md](README_NAV.md)**
