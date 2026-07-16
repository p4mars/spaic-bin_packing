# Node-Level Instructions for driving - Guilherme

**Package:** `mirte_driving` · **Nodes:**  `zone_detector`,
`shuttle_manager` · **Launch files:** `shuttle.launch.py`, `mission.launch.py`,
`detector.launch.py`.

> Build + source
> ```bash
> cd ~/spatial-ai/ws && colcon build --packages-select mirte_driving --symlink-install
> source install/setup.bash
> ```
> Run node `ros2 run mirte_driving <executable> --ros-args -p name:=value …`.

---


## `zone_detector` — camera → goal poses  *(NAVIGATION input)*

**Executable:** `zone_detector.py` · **Node name:** `zone_detector`

**Purpose:** the only node that touches the camera. Detects ArUco markers with
OpenCV (`cv2.aruco`), converts each to a pose in the `map` frame via TF, smooths
it, and publishes Zone A (one marker) and Zone B (midpoint of a marker pair) as the
goals the mission drives to. Runs **on the robot or on the laptop** — same node,
same output topics.

**Run standalone** (values shown are the real/sim-arena scheme; node defaults are
the old sim values `DICT_4X4_50`, ids 0/1/2):
```bash
ros2 run mirte_driving zone_detector.py --ros-args \
  -p aruco_dict:=DICT_4X4_250 -p zone_a_id:=100 \
  -p zone_b_left_id:=101 -p zone_b_right_id:=102 -p zone_marker_size:=0.08
```

| Subscribes | Type | Notes |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | when `use_compressed:=false` |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | when `use_compressed:=true` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | intrinsics (or load from `camera_info_path`) |
| (TF) `map ← camera_optical_frame` | — | needed to place markers in `map` — **depends on SLAM** |

| Publishes | Type | Notes |
|---|---|---|
| `/zone_a_pose` | `geometry_msgs/PoseStamped` (`map`) | single marker `zone_a_id`; re-published at 5 Hz |
| `/zone_b_pose` | `geometry_msgs/PoseStamped` (`map`) | midpoint of `zone_b_left_id`+`zone_b_right_id` |

| Parameter | Default | Meaning |
|---|---|---|
| `aruco_dict` | `DICT_4X4_50` | OpenCV ArUco dictionary name (mission uses `DICT_4X4_250`) |
| `zone_a_id` | `0` | marker id for Zone A (mission: `100`) |
| `zone_b_left_id` | `1` | left marker of the Zone B pair (mission: `101`) |
| `zone_b_right_id` | `2` | right marker of the Zone B pair (mission: `102`) |
| `zone_marker_size` | `0.08` | printed marker side length in metres (sim arena: `0.15`) |
| `camera_frame` | `camera_color_optical_frame` | frame used for the marker→map transform |
| `camera_info_path` | `''` | optional path to a `camera_info.yaml` (skips waiting on the topic over wifi) |
| `frame_skip` | `5` | process every Nth frame (CPU saver) |
| `use_compressed` | `False` | subscribe to the compressed image stream instead of raw |

**Depends on:** the camera driver (`/camera/image_raw`, `/camera/camera_info`) **and
SLAM** (for the `map ← camera` transform — no transform ⇒ nothing is published).
**Verify:** `ros2 topic echo /zone_a_pose --once` (needs the marker in view + SLAM up).

---

## `shuttle_manager` — the mission brain (FSM)  *(NAVIGATION)*

**Executable:** `shuttle_manager.py` · **Node name:** `shuttle_manager`

**Purpose:** the finite-state machine `WAIT_SLAM → SEARCH → SHUTTLE → DONE`. It reads
the zone poses, sends `NavigateToPose` goals to Nav2 for each leg, drives the wheels
directly for the search spin / camera fine-alignment / 180° turn, tunes Nav2's
inflation and speed live, and hands off to the Zone A grasp and Zone B dock. It does
**no** SLAM/planning/avoidance itself.

**Run standalone** (it only *acts* once SLAM, Nav2, and the detector are up — on its
own it just sits in `WAIT_SLAM`):
```bash
ros2 run mirte_driving shuttle_manager.py --ros-args \
  -p round_trips:=2 -p arm_mimic:=true -p dock_at_b:=false -p grasp_at_a:=false
```

### Inputs / outputs

| Subscribes | Type | From |
|---|---|---|
| `/zone_a_pose`, `/zone_b_pose` | `geometry_msgs/PoseStamped` | `zone_detector` |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` |
| (TF) `map → base_link` | — | SLAM + base (localisation) |
| `/robot_positioned`, `/robot_backed_up`, `/robot_turned_around`, `/navigation_failed` | `std_msgs/Bool` | Zone-B `marker_navigator` |
| `/box_placed` | `std_msgs/String` | Zone-B `box_placer` |
| `/perception/object_markers` | `visualization_msgs/MarkerArray` | Zone-A `perception_node` (laptop) |

| Publishes | Type | To |
|---|---|---|
| `cmd_vel_topic` (default `/mirte_base_controller/cmd_vel_unstamped`) | `geometry_msgs/Twist` | base — direct drive (spin/align/turn) |
| `/mirte_master_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | arm poses |
| `/start_placing` | `std_msgs/Bool` | triggers Zone-B `box_placer` |

| Action / Service (as client) | Type | Target |
|---|---|---|
| `navigate_to_pose` (action) | `nav2_msgs/action/NavigateToPose` | Nav2 `bt_navigator` — every leg |
| `/mirte_master_gripper_controller/gripper_cmd` (action) | `control_msgs/action/GripperCommand` | gripper |
| `/grasp_handle` (service) | `std_srvs/srv/Trigger` | Zone-A `grasp_node` (laptop) |
| `/{global,local}_costmap/…/set_parameters` (service) | `rcl_interfaces/srv/SetParameters` | per-leg inflation |
| `/controller_server/set_parameters` (service) | `rcl_interfaces/srv/SetParameters` | live cruise speed |

### Parameters (grouped; all have `declare_parameter` defaults)

**Mission / motion**

| Parameter | Default | Meaning |
|---|---|---|
| `round_trips` | `3` | number of A→B→A round trips |
| `approach_dist` | `0.25` | generic stand-off distance from a zone (m) |
| `approach_dist_a` | `1.0` | Zone A stand-off (m) — the align servo then refines |
| `shuttle_speed` | `0.32` | cruise speed once both zones are found (m/s) |
| `search_angular` | `0.3` | search-spin rate (rad/s) — gentle to keep SLAM locked |
| `search_spin_time` | `17.0` | seconds for one search sweep before wandering |
| `relocate_dist` | `1.5` | wander hop distance to a new vantage (m) |
| `relocate_timeout` | `30.0` | s before a stalled wander drive is cancelled |
| `goal_timeout` | `60.0` | s before a leg's Nav2 goal is cancelled/retried |
| `slam_wait_timeout` | `60.0` | s to wait for SLAM in `WAIT_SLAM` before aborting |
| `cmd_vel_topic` | `/mirte_base_controller/cmd_vel_unstamped` | where the node writes `Twist` |

**Alignment / turn**

| Parameter | Default | Meaning |
|---|---|---|
| `align_at_a`, `align_at_b` | `True` | run the camera fine-alignment servo at each zone |
| `align_timeout` | `30.0` | s before a stuck alignment gives up |
| `turn_at_b` | `True` | spin 180° after the Zone B stage |

**Arm / gripper (mimic)**

| Parameter | Default | Meaning |
|---|---|---|
| `arm_mimic` | `True` | do the carry/drop arm poses (no real grasp) |
| `arm_up_angles` | `[0,0,0,0]` | upright pose `[pan, lift, elbow, wrist]` (rad) |
| `arm_box_angles` | `[0,-1.2,-1.5,1.4]` | box-holding pose |
| `arm_grab_angles` | `[0,-0.4329,-0.8916,-0.3]` | pose to grab a box at A (dock mode) |
| `arm_zero_angles` | `[0,0,0,0]` | arm home |
| `arm_hold_period` | `4.0` | s between re-sends that keep the arm energised |
| `gripper_open_pos` / `gripper_close_pos` | `-0.6` / `0.5` | gripper open/close positions (rad) |

**Zone B — precise dock / place**

| Parameter | Default | Meaning |
|---|---|---|
| `dock_at_b` | `True` | spawn the Zone-B package at B (precise dock + place) |
| `placement_package` | `mirte_placement` | package to spawn — **⚠ set to `mirte_stacking` in this repo (see note below)** |
| `dock_approach_dist` | `0.5` | Zone B stand-off when docking (m) |
| `dock_wait_for_box` | `False` | run the full lay-down/return cycle (else adjust-only) |
| `auto_start_placing` | `True` | auto-trigger `box_placer` on `/robot_positioned` |
| `dock_marker_left` / `dock_marker_right` | `101` / `102` | the Zone B marker pair |
| `dock_marker_size` | `0.08` | Zone B marker size (m) |
| `dock_timeout` | `240.0` | s before the dock is abandoned |
| `dock_image_topic` / `dock_info_topic` | `/camera/color/image_raw` / `…/camera_info` | camera topics passed to `marker_navigator` |
| `dock_cmd_vel_topic` | `/mirte_base_controller/cmd_vel` | where `marker_navigator` should drive |
| `dock_approach_m`, `dock_seek_dist` | `-1.0`, `-1.0` | `<0` = use `marker_navigator`'s own defaults |

**Zone A — handle grasp (laptop)**

| Parameter | Default | Meaning |
|---|---|---|
| `grasp_at_a` | `False` | hand A off to the laptop's `mirte_perception` grasp |
| `grasp_detect_timeout` | `30.0` | s to wait for a handle detection before skipping |
| `grasp_timeout` | `180.0` | overall cap on the grasp (incl. the 30–120 s service) |

**Costmap inflation (live)**

| Parameter | Default | Meaning |
|---|---|---|
| `dynamic_inflation` | `True` | change inflation per leg via the costmap param service |
| `inflation_carry` | `0.45` | inflation radius on the A→B (arm-out) leg (m) |
| `inflation_empty` | `0.28` | inflation radius on the B→A (empty) leg (m) |

**Depends on:** SLAM (`/map` + `map→base_link`), Nav2 (`navigate_to_pose` active),
`zone_detector` (the pose topics). For `dock_at_b`: the Zone-B package. For
`grasp_at_a`: the laptop's `mirte_perception`.
**Verify:** `ros2 topic echo /mirte_base_controller/cmd_vel_unstamped` (it's driving)
and watch the log states `WAIT_SLAM → SEARCH → SHUTTLE`.

> **⚠ Repo note — package naming:** this repository's Zone-B package is
> **`mirte_stacking`**, but `shuttle_manager`'s `placement_package` defaults to
> `mirte_placement` (and the README uses `mirte_placement` throughout). So
> `dock_at_b:=true` will fail to spawn unless you pass
> `-p placement_package:=mirte_stacking`. Fix one of the two names so they agree.

---

## Launch files (how the nodes are started together)

You normally start the nodes through a launch file, not one by one.

| Launch | Runs on | Starts | Use |
|---|---|---|---|
| `shuttle.launch.py` | robot / sim | `scan_filter`, `slam_toolbox`, Nav2 (composed container), `shuttle_manager`, and `zone_detector` (if `run_zone_detector:=true`) | the whole stack; **sim defaults** (`use_sim_time:=true`, `/cmd_vel`) |
| `mission.launch.py` | real robot | includes `shuttle.launch.py` with real-robot args (`run_zone_detector:=false`) | one-command real-robot run; detector offloaded to the laptop |
| `detector.launch.py` | **laptop** | `zone_detector` only | run detection off-robot; publishes `/zone_a_pose`, `/zone_b_pose` over wifi |

**Sim (one command):**
```bash
ros2 launch mirte_driving shuttle.launch.py dock_at_b:=false grasp_at_a:=false arm_mimic:=true round_trips:=2
```
**Real robot:**
```bash
ros2 launch mirte_driving mission.launch.py         # on the robot
ros2 launch mirte_driving detector.launch.py        # on the laptop (same ROS_DOMAIN_ID)
```

Common `mission.launch.py` / `shuttle.launch.py` arguments: `round_trips`,
`arm_mimic`, `dock_at_b`, `grasp_at_a`, `run_zone_detector`, `scan_min_range`,
`align_at_a`/`align_at_b`, and the ArUco settings (`aruco_dict`, `zone_a_id`,
`zone_b_left_id`, `zone_b_right_id`, `zone_marker_size`). These forward to the node
parameters above.

---

## Quick node map

```
lidar ─/scan─► scan_filter ─/scan_filtered─► slam_toolbox ─/map,map→odom─► Nav2 costmaps
camera ─────► zone_detector ─/zone_a_pose,/zone_b_pose─► shuttle_manager ─NavigateToPose─► Nav2 ─Twist─► wheels
                                                          shuttle_manager ─Twist─► wheels (spin/align/turn)
                                                          shuttle_manager ↔ Zone-A grasp (/grasp_handle) / Zone-B dock (handshake topics)
```
