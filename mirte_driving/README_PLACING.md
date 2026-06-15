# Box Placing — marker_navigator + box_placer

## Overview

Two nodes work together to place a box held by the robot arm into a bin marked
by two ArUco markers:

- **`marker_navigator`** — drives the robot to the correct position in front of
  the bin, coordinates the drive-back after the arm lowers, then turns the robot
  180° to hand off to the next script.
- **`box_placer`** — controls the arm and gripper: lowers the box, holds while
  the robot backs away, repositions the wrist, releases the box, and raises the
  arm ready for the next cycle. Tracks and persists a stack height offset so each
  successive box is placed 3 cm higher.

Neither node controls the other directly. They communicate exclusively through
ROS 2 topics.

---

## Node Communication

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          HARDWARE / BRINGUP                             │
│  /camera/color/image_raw  ──────────────────────────────────────────►  │
│  /camera/color/camera_info ─────────────────────────────────────────►  │
│  TF: odom → base_link                                                   │
│      odom → camera_color_optical_frame                                  │
│  /joint_states ──────────────────────────────────────────────────────►  │
│  /mirte_master_arm_controller/follow_joint_trajectory  (action server)  │
│  /mirte_master_gripper_controller/gripper_cmd          (action server)  │
└─────────────────────────────────────────────────────────────────────────┘
          │                                          │
          ▼                                          ▼
┌──────────────────────┐               ┌──────────────────────┐
│   marker_navigator   │               │      box_placer      │
│                      │               │                      │
│  SUBSCRIBES:         │               │  SUBSCRIBES:         │
│  /camera/color/      │               │  /robot_positioned   │
│    image_raw         │               │  /robot_backed_up    │
│  /camera/color/      │               │  /start_placing      │
│    camera_info       │               │  /reset_stack        │
│  /arm_placed  ◄──────┼───────────────┼──────────────────    │
│  /box_placed  ◄──────┼───────────────┼──────────────────    │
│                      │               │  /joint_states       │
│  PUBLISHES:          │               │                      │
│  /robot_positioned ──┼───────────────┼──────────────────►   │
│  /robot_backed_up ───┼───────────────┼──────────────────►   │
│  /robot_turned_      │               │  PUBLISHES:          │
│    around            │               │  /arm_placed ────────┼──► marker_navigator
│  /navigation_failed  │               │  /box_placed ────────┼──► marker_navigator
│  /mirte_base_        │               │  /place_failed       │
│    controller/       │               │  /box_placed ────────┼──► (next script)
│    cmd_vel           │               │                      │
│  /aruco_101_pose     │               │  ACTION CLIENTS:     │
│  /aruco_102_pose     │               │  follow_joint_       │
│                      │               │    trajectory        │
│  NO action servers   │               │  gripper_cmd         │
└──────────────────────┘               └──────────────────────┘
```

### Key design points

- **marker_navigator uses no action servers.** All robot motion is open-loop
  `Twist` messages on `/mirte_base_controller/cmd_vel`. The node closes the
  control loop itself by reading odometry (via TF) and camera feedback every
  50 ms.
- **box_placer uses two action servers** — one for the arm
  (`FollowJointTrajectory`) and one for the gripper (`GripperCommand`). It
  sends joint-space trajectories and waits for action results before advancing.
- **Latched topics** (`/robot_positioned`, `/robot_backed_up`,
  `/robot_turned_around`, `/navigation_failed`, `/arm_placed`, `/place_failed`)
  are retained by the broker so a subscriber that starts late still receives the
  last published value.

---

## Full Grasp Procedure — Step by Step

### Phase 1 — Startup

1. Start the Mirte bringup on the robot (camera, arm controller, base
   controller, TF tree).
2. Manually move the arm to the carry position and open the gripper (see
   **Launch** section below).
3. Place the box in the open gripper.
4. Start `marker_navigator` — it immediately begins scanning for markers.
5. Start `box_placer` in a second terminal — it sits in IDLE waiting.

---

### Phase 2 — Navigation (marker_navigator)

**SEARCHING**
- Robot sweeps ±30° around its starting heading.
- Each camera frame (every 5th frame) is converted to grayscale and run through
  the ArUco detector.
- When a marker is found, `estimatePoseSingleMarkers` computes its 3D position
  in camera space. The TF chain
  `odom → camera_color_optical_frame` converts this to odom coordinates.
- Positions are EMA-smoothed (α = 0.25) to reduce noise.
- Once **both** markers appear in the **same frame**, the target is computed.

**Target geometry (`_compute_target`)**
- Midpoint M between the two markers in odom space.
- Perpendicular to the marker line gives the approach direction.
- Dot product with the robot position picks the correct side.
- `target_x`, `target_y` = M + 30 cm along the approach direction.
- `target_yaw` = direction facing back toward the bins.
- Target is **locked** — not recomputed unless both markers appear together again.

**DRIVE**
- P-controller on all three axes simultaneously (mecanum drive can strafe):
  ```
  linear.x  = KP_LIN × (forward component of error)
  linear.y  = KP_LIN × (strafe component of error)
  angular.z = KP_ANG × (angle to target − robot yaw)
  ```
- Published to `/mirte_base_controller/cmd_vel` at 20 Hz.
- Target only recomputed when both markers visible in same frame.
- Stops when distance to target ≤ 3 cm.

**STOP**
- Zero velocity for 1 second to let the robot settle.

**ROTATE**
- Pure in-place yaw correction to `target_yaw`.
- Target NOT recomputed here — locks the yaw from the drive phase.
- Accepts ≤ 5° error (mecanum stalls below ~4°).
- 30-second timeout: accepts current yaw and continues regardless.

**DONE**
- Publishes `/robot_positioned` (latched).
- box_placer receives this and auto-starts (if `auto_start=True`).
- marker_navigator stops all movement and waits for `/arm_placed`.

---

### Phase 3 — Arm lowers (box_placer)

**CLOSE_GRIP**
- Sends `GripperCommand` goal: position = 0.35 rad, effort = 10 N.
- Waits a fixed 7 seconds (timer-based, action result ignored — stalling on
  the box is expected and does not indicate failure).

**PLACE_DOWN**
- Sends `FollowJointTrajectory` to `POSE_PLACE` (shifted up by the current
  stack offset) with duration 12 seconds.
- Stack offset = (boxes placed so far) × 3 cm, converted to joint space via a
  linear approximation.
- Waits for the action result callback.

**PLACE_WAIT**
- Reads `/joint_states` every tick to verify the arm actually reached the target
  (within 0.12 rad on each joint).
- If verified → publishes `/arm_placed` (latched) → marker_navigator starts
  driving back.
- If not verified and action already returned → resends trajectory.
- Gives up after `T_PLACE_DOWN + 15 s` and publishes `/arm_placed` anyway.

---

### Phase 4 — Drive-back (marker_navigator)

**DRIVE_BACK**
- Receives `/arm_placed`, transitions immediately.
- P-controller in reverse:
  ```
  error = distance_to_midpoint − seek_dist_m
  vel   = SEEK_KP × error          (negative = backward)
  linear.x = vel,  angular.z = 0   (straight back, no steering)
  ```
- Distance measured as 2D Euclidean from `base_link` to marker midpoint in odom.
- Stops when within 5 mm of `seek_dist_m` (default 50 cm).
- **Fallback:** if markers are lost, switches to odometry — drives back 25 cm
  from where the fallback started.
- Hard timeout: 60 seconds regardless of distance.

**BACKED_UP**
- Publishes `/robot_backed_up` (latched).
- Waits for `/box_placed` before turning around.

---

### Phase 5 — Wrist reposition and release (box_placer)

**WAIT_FOR_BACK** (box_placer was waiting here during Phase 4)
- Receives `/robot_backed_up`.
- Immediately sends arm trajectory: wrist sweeps from `WRIST_PLACE` to
  `WRIST_BACK` (−0.6 rad), elbow raised by `ELBOW_BACK_DELTA` (0.15 rad),
  over `T_WRIST_BACK` seconds (10 s).
- Transitions to REPOSITION.

**REPOSITION**
- Waits for the wrist sweep action result (≈ 10 s).

**OPEN_GRIP**
- Sends `GripperCommand`: position = −0.3 rad.
- Waits 7 seconds (timer-based).

**SETTLING**
- Increments `box_count`, adds 3 cm to `stack_z_offset`.
- Saves both to `~/.mirte_stack_state.json` (persists across restarts).
- Waits 1.5 seconds.

**RETURN_HOME**
- Sends arm to `POSE_CARRY` (already shifted for the next box's stack height).
- Duration 7 seconds.

**IDLE**
- Publishes `/box_placed` (String: `"box_1"`, `"box_2"`, …).
- Ready for the next box.

---

### Phase 6 — Turn-around (marker_navigator)

**TURN_AROUND**
- Receives `/box_placed`.
- Reads current yaw from TF, sets `turn_target = yaw + 180°`.
- P-controller rotates in place until within 5°.
- Fallback if no odom: timed rotation at MAX_ANG for π / MAX_ANG seconds.
- 45-second hard timeout.

**FINISHED**
- Publishes `/robot_turned_around` (latched).
- Stops publishing to `/cmd_vel` entirely — leaves it free for the next script.

---

## Signal Flow Summary

```
marker_navigator                            box_placer
────────────────                            ──────────
DONE: /robot_positioned ─────────────────► auto-start → CLOSE_GRIP
                                                         PLACE_DOWN
                                                         PLACE_WAIT
      ◄──────────────────── /arm_placed ── (arm lowered, verified)
DRIVE_BACK
      /robot_backed_up ──────────────────► WAIT_FOR_BACK → REPOSITION
                                                            OPEN_GRIP
                                                            SETTLING
                                                            RETURN_HOME
      ◄────────────────────── /box_placed  IDLE (publishes)
TURN_AROUND
FINISHED: /robot_turned_around ──────────► (next script)
```

---

## Failsafes

| Situation | What happens |
|---|---|
| Markers not found within 120 s | FAILED — robot stops, `/navigation_failed` published; auto-resumes if markers reappear |
| Markers lost mid-drive (> 10 s) | Clears positions, returns to SEARCHING |
| Approach takes > 120 s | FAILED |
| Yaw correction won't converge (> 30 s) | Accepts current yaw and continues |
| `/arm_placed` never arrives | Warning every 30 s, waits indefinitely |
| Drive-back > 60 s | Finishes anyway, publishes `/robot_backed_up` |
| Drive-back markers lost | Odom fallback (25 cm), then finishes |
| `/robot_backed_up` never arrives | box_placer re-publishes `/arm_placed` at 60 s; force-releases box at 120 s |
| Any box_placer state stuck > 180 s | Aborts to IDLE, publishes `/place_failed` |
| Arm action no result within 30 s | Force-continues |
| Arm goal rejected 5× | Force-continues |

---

## How to Launch

### Step 0 — Prerequisites

Mirte bringup must already be running on the robot:
```bash
# (runs automatically on Mirte boot, or manually:)
ros2 launch mirte_bringup mirte_master.launch.py
```

Verify the arm controller and gripper controller are active:
```bash
ros2 control list_controllers | grep -E "arm|gripper"
```

### Step 1 — Move arm to carry position

```bash
ros2 action send_goal /mirte_master_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, \
elbow_joint, wrist_joint], points: [{positions: [0.0, -0.4329, -0.8916, -0.3], \
time_from_start: {sec: 3, nanosec: 0}}]}}"
```

### Step 2 — Open gripper

```bash
ros2 action send_goal /mirte_master_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: -0.3, max_effort: 10.0}}"
```

### Step 3 — Place box in gripper

Hold the box against the open gripper fingers.

### Step 4 — Start marker_navigator (Terminal 1)

```bash
python3 ~/mirte_ws/src/group3_spatial-ai/mirte_workshop/marker_navigator.py \
  2>&1 | grep -v "TF_NAN_INPUT\|TF_DENORMALIZED\|buffer_core"
```

The robot will begin scanning for ArUco markers 101 and 102. Wait until you see:
```
Robot positioned!
```

### Step 5 — Start box_placer (Terminal 2)

```bash
python3 ~/mirte_ws/src/group3_spatial-ai/mirte_workshop/box_placer.py
```

With `auto_start=True` (default) box_placer starts automatically when
`/robot_positioned` arrives. For manual control:

```bash
ros2 topic pub --once /start_placing std_msgs/msg/Bool '{data: true}'
```

### Step 6 — Repeat for subsequent boxes

After `/robot_turned_around` is published the robot faces away from the bins.
Bring the next box, open the gripper, place the box, and re-trigger
`/start_placing` (or restart marker_navigator for a full navigation cycle).

### Reset stack counter

If you want to restart from box 1 (place height back to zero):
```bash
ros2 topic pub --once /reset_stack std_msgs/msg/Bool '{data: true}'
# or:
rm ~/.mirte_stack_state.json
```

### Skip-approach mode (testing drive-back only)

Position the robot manually in front of the bins, then:
```bash
python3 marker_navigator.py --ros-args -p skip_approach:=true
```
Publishes `/robot_positioned` immediately without driving, then waits for
`/arm_placed` and handles drive-back normally.

---

## Tuning Reference

### marker_navigator

| Parameter | Default | Notes |
|---|---|---|
| `approach_m` | 0.30 m | Distance from base_link to marker midpoint at stop |
| `seek_dist_m` | 0.50 m | Drive-back target — must be > `approach_m` |
| `fallback_back_m` | 0.25 m | Odom fallback if markers lost during drive-back |
| `marker_size` | 0.08 m | Physical side length of ArUco markers — wrong value breaks all distance estimates |
| `scan_vel` | 0.25 rad/s | Sweep speed during SEARCHING |
| `map_frame` | `odom` | Real robot has no `/map` — do not change |

### box_placer

| Constant | Default | Notes |
|---|---|---|
| `POSE_CARRY` | `[0.0, -0.4329, -0.8916, -0.3]` | Arm holding position — tune by trial and error |
| `POSE_PLACE` | `[0.0, -1.2, -0.8916, -0.3]` | Arm fully lowered — tune until gripper sits at bin floor level |
| `WRIST_BACK` | `-0.6 rad` | Wrist angle after drive-back for box clearance |
| `ELBOW_BACK_DELTA` | `0.15 rad` | Elbow raise during wrist reposition |
| `T_WRIST_BACK` | `10.0 s` | Duration of wrist reposition sweep |
| `T_PLACE_DOWN` | `12.0 s` | Time given to arm controller for descent |
| `T_RETURN` | `7.0 s` | Time given to arm controller to raise back to carry |
| `GRIP_DURATION` | `7.0 s` | Fixed wait after any gripper command |
| `GRIPPER_OPEN` | `-0.3 rad` | Robot-specific — verify on each robot |
| `GRIPPER_CLOSED` | `0.35 rad` | Increase if box slips |
| `GRIP_EFFORT` | `10.0 N` | Increase if gripper won't close |
| `BOX_HEIGHT_STEP` | `0.030 m` | Height added per placed box |
| `PLACE_TRAVEL_HEIGHT_M` | `0.40 m` | Estimated vertical travel carry→place — used to scale stack offset into joint space |

---

## Files

```
mirte_workshop/
├── marker_navigator.py   — navigation, drive-back, turn-around
├── box_placer.py         — arm and gripper control, stack tracking
└── camera_info.yaml      — camera intrinsics (optional, falls back to topic)

~/.mirte_stack_state.json — persisted box count and stack height (auto-created)
```
