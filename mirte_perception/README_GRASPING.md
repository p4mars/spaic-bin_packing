# mirte_perception

ROS2 package for door-handle detection and grasping on the Mirte Master robot. It contains two nodes — `perception_node` and `grasp_node` — that together form a full pick-up pipeline: detect a handle with a depth camera, align the robot base using a top-down gripper camera, lower the arm and close the gripper.

---

## Package structure

```
mirte_perception/
├── mirte_perception/
│   ├── perception_node.py
│   └── grasp_node.py
├── models/
│   ├── handles_model.pt      # YOLO weights for handle detection (front camera)
│   └── gripper_model.pt      # YOLO weights for top-down gripper camera
├── launch/
│   ├── perception.launch.py  # starts perception_node only
│   └── grasp.launch.py       # starts both nodes (full pipeline)
└── package.xml / setup.py
```

---

## Nodes

### `perception_node`

The "eyes" of the robot. Runs continuously, subscribing to the front depth camera, detecting box handles with YOLO, computing their 3D position and publishing that position for other nodes to consume.

#### Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics (fx, fy, cx, cy). Saved once on first message. |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth frame. Latest is stored; processed when a colour frame arrives. |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Colour frame. Triggers the main detection pipeline on every frame. |

#### Publishers

| Topic | Type | Purpose |
|---|---|---|
| `/perception/annotated_image` | `sensor_msgs/Image` | Colour image with bounding boxes and depth labels drawn on it. For debugging in RViz or `rqt_image_view`. |
| `/perception/object_markers` | `visualization_msgs/MarkerArray` | 3D sphere + text markers at the detected handle positions, expressed in `camera_depth_optical_frame`. Consumed by `grasp_node`. |

#### Parameters

| Name | Default | Description |
|---|---|---|
| `model1_path` | *(required)* | Path to YOLO `.pt` weights for handle detection. Node crashes on startup if not set. |
| `confidence_threshold` | `0.7` | Minimum YOLO confidence to keep a detection. |
| `fixed_frame` | `camera_depth_optical_frame` | TF frame for 3D marker positions. |
| `handle_real_width_m` | `0.02` | Known physical width of a handle in metres. Used as depth fallback. |

#### What it does per frame

1. Runs YOLO on the colour image.
2. For each detection above the confidence threshold, samples a 9×9 pixel patch from the depth image at the bounding-box centre and takes the median of valid pixels as the depth reading.
3. If the depth sensor returns nothing valid, falls back to estimating depth from the apparent bounding-box width using the pinhole formula: `depth = (fx × real_width) / pixel_width`. These estimated detections appear yellow in RViz.
4. Projects each depth + pixel pair into a 3D point using back-projection: `X = (u − cx) × Z / fx`, `Y = (v − cy) × Z / fy`, `Z = depth`.
5. Publishes a sphere marker at that 3D position (green = measured depth, yellow = estimated) and a text label above it.

---

### `grasp_node`

The "hands" of the robot. Waits idle until the `/grasp_handle` service is called, then executes the full grasp sequence: move the arm to a hover pose, drive the base using the top-down gripper camera until the handle is centred, lower the arm and close the gripper.

#### Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `/perception/object_markers` | `visualization_msgs/MarkerArray` | 3D handle positions from `perception_node`. Updated continuously even during the grasp. |
| `/gripper_camera/image_raw` | `sensor_msgs/Image` | Top-down camera mounted on the gripper. Used for visual servoing. Latest frame stored. |
| `/gripper_camera/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics for the gripper camera. Used to convert pixel error to metres. |

#### Publishers

| Topic | Type | Purpose |
|---|---|---|
| `/mirte_master_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Commands the arm to a set of joint angles within a time budget. |
| `/mirte_base_controller/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to the holonomic base during visual servoing. Publishing all-zeros stops the base. |
| `/grasp/proceed` | `std_msgs/Bool` | Published `True` once when a grasp is confirmed. Signals downstream behaviour to proceed. |
| `/gripper_camera/annotated_image` | `sensor_msgs/Image` | Debug image from the gripper camera with the detected handle centroid and target point drawn on it. |

#### Action clients

| Server | Type | Purpose |
|---|---|---|
| `/mirte_master_gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | Opens and closes the gripper. Returns the actual final joint position, used to detect whether something was caught. |

#### Services provided

| Service | Type | Description |
|---|---|---|
| `/grasp_handle` | `std_srvs/Trigger` | Trigger the full grasp sequence. Blocks until the grasp succeeds or all retries are exhausted. Returns `success=True/False` and a message. |

#### Parameters

| Name | Default | Description |
|---|---|---|
| `gripper_model_path` | `""` | Path to YOLO weights for top-down handle detection. If not set, visual servoing is disabled. |

---

## How the nodes interact

```
Camera driver
  ├─ /camera/color/image_raw    ──► perception_node
  ├─ /camera/depth/image_raw    ──► perception_node
  └─ /camera/depth/camera_info  ──► perception_node

perception_node
  ├─ /perception/annotated_image  ──► RViz (debug)
  └─ /perception/object_markers   ──► grasp_node

Gripper camera driver
  ├─ /gripper_camera/image_raw    ──► grasp_node
  └─ /gripper_camera/camera_info  ──► grasp_node

grasp_node
  ├─ /mirte_master_arm_controller/joint_trajectory  ──► arm controller
  ├─ /mirte_base_controller/cmd_vel                 ──► base controller
  ├─ /gripper_camera/annotated_image                ──► RViz (debug)
  └─ /grasp/proceed                                 ──► downstream behaviour

External caller ──► /grasp_handle (service) ──► grasp_node
```

`perception_node` runs independently and continuously. `grasp_node` subscribes to its output but only acts when `/grasp_handle` is called. The marker subscription remains active during the grasp (via `ReentrantCallbackGroup` + `MultiThreadedExecutor`) so the node keeps receiving fresh handle positions even while the arm is moving.

---

## Full grasp pipeline

This describes what happens from the moment `/grasp_handle` is called.

### Pre-checks

Before moving anything, `grasp_node` checks:
- At least one handle has been detected by `perception_node` (i.e., `/perception/object_markers` has been received at least once).
- The most recent detection is less than 3 seconds old. Stale data from a handle that may no longer be in view is rejected.
- The handle position can be transformed from `camera_depth_optical_frame` to `base_link` via TF2. If the TF tree is missing or stale, the grasp is aborted.

If any check fails, the service returns immediately with `success=False`.

### Step 1 — Average depth readings and transform to `base_link`

The last 5 handle detections from `perception_node` are averaged in X and Y before the transform. Depth readings are noisy frame-to-frame; averaging gives a more stable initial target. The averaged position is then transformed by TF2 into `base_link` (the robot's floor-level frame where arm movements are planned).

### Step 2 — Open gripper and move arm to hover pose

The gripper is opened first to avoid accidentally dragging anything. The arm then moves to a fixed hover configuration (`_HOVER_JOINTS`): a pre-calibrated set of joint angles that points the gripper camera straight down at approximately 20 cm above the floor. This pose was found empirically.

The arm is commanded by publishing a `JointTrajectory` message with a 7-second execution window, then sleeping to wait for completion. There is no feedback; the sleep time is generous enough that the arm reliably arrives.

### Step 3 — Visual servo (base alignment)

With the arm stationary in the hover pose, the robot base is driven to centre the handle under the gripper using the top-down gripper camera and a second YOLO model.

The servo loop runs up to 300 iterations. Each iteration:

1. **Waits for a genuinely new frame** from the gripper camera (different timestamp than the last one used) to avoid acting on stale images.
2. **Runs YOLO** on the frame to find the handle centroid `(cx, cy)`.
3. **Computes pixel error** relative to the target point: horizontally centred, 85% of the way down the frame (because the camera is mounted at the front of the gripper, so the handle should appear near the bottom when the gripper is directly above it).
4. **Converts pixel error to metres** using the pinhole formula: `err_m = err_px × hover_height / focal_length`.
5. **Drives the base**:
   - *Coarse phase* (error > 10 cm): drives at a fixed fast speed (`_SERVO_FAST_VEL = 0.06 m/s`) for an estimated duration, open-loop.
   - *Fine phase* (error ≤ 10 cm): proportional control (`velocity = Kp × error`), clamped between a minimum dead-zone velocity and a maximum safe velocity.
6. **Convergence**: when both X and Y errors are under 20 pixels, the base stops, waits 2 seconds to settle, then re-runs YOLO on a fresh frame to confirm the alignment held. Up to 3 confirmation failures resume the servo.

#### Robustness guards inside the servo

| Condition | Action |
|---|---|
| YOLO misses ≤ 3 consecutive frames | Reuse last known centroid (smooths transient misses) |
| YOLO misses > 5 consecutive frames | Depth camera nudge (see below) |
| Handle more than 35% of frame width off-centre | Depth camera nudge |
| Servo error not improving for 20 iterations | Depth camera nudge |
| Handle below 95% of frame height (robot too close) | Back up 2 cm immediately |
| More than 7 depth camera nudges total | Give up visual servo, hand off to depth-camera fallback |
| 300 iterations without convergence | Give up with `exit_reason = 'timeout'` |

**Depth camera nudge** (`_depth_camera_nudge`): when the gripper camera loses the handle, the node falls back to the last known depth-camera marker. It transforms that position to `base_link`, computes where the gripper currently is using forward kinematics of the hover joints, and drives the base at half the metric error (gain = 0.5) for 0.3 seconds to re-centre.

### Step 3b — Forward nudge after convergence

After the visual servo converges, the robot nudges forward 5.5 cm. This corrects for the fact that the gripper camera is mounted slightly ahead of the gripper jaw: "camera centred on handle" is not the same as "jaw over handle."

### Step 4 — Compute grasp position via forward kinematics

Because the base has moved during visual servo, the original TF-based handle position is stale. Instead of re-detecting the handle, the node uses forward kinematics (FK) of the hover joints to find where the gripper currently sits in `base_link`. This point (same X/Y as the gripper, but with Z=`_GRASP_Z_M`) becomes the IK target.

The gripper orientation from the hover pose (pointing straight down) is extracted from the FK result and passed to IK as a constraint, so the gripper stays pointing down during the descent.

### Step 5 — Lower arm via inverse kinematics

Inverse kinematics (IK) is solved for the grasp position using the `ikpy` library, which reads the robot's URDF. Multiple initial seeds are tried and the one with the smallest forward-kinematics error is selected. A solution with residual error > 8 cm is rejected as unreliable.

The arm is then commanded to the IK solution (7-second trajectory), lowering the gripper to the handle height.

### Step 6 — Close gripper and lift

The gripper closes via a `GripperCommand` action. The action returns the actual final joint position of the gripper (the position where it stopped). The arm then lifts to 25 cm above the floor using a second IK solution.

### Step 7 — Verify grasp

The gripper's reported final position is compared against the fully-closed value (`0.3502 rad`). If it stopped more than 0.08 rad short of fully closing, something is between the jaws — the grasp is confirmed. If it closed all the way, the fingers met: nothing was caught.

On **success**: publishes `True` on `/grasp/proceed`, returns `success=True` from the service.

On **slip/miss**: opens the gripper, backs up 10 cm, returns arm to hover and retries from Step 3. Up to 3 total attempts are made.

### Depth-camera fallback (when visual servo gives up)

If the visual servo exits because of too many depth-camera nudges (`exit_reason = 'depth_available'`), a separate path is taken instead of retrying:

1. Drive the base toward the handle (from depth camera history) until the handle is within 25 cm, or until it disappears from the depth camera (robot is now close enough).
2. Average the last 5 depth readings for a stable IK target.
3. Run IK and execute the grasp (Steps 5–7) directly, without a gripper-camera visual servo.

### Pipeline summary

```
/grasp_handle called
        │
        ▼
Pre-checks: handle seen recently? TF available?
        │
        ▼
Average last 5 depth readings → transform to base_link
        │
        ▼
Open gripper
        │
        ▼
Move arm to fixed hover pose (camera facing down)
        │
        ┌────────────────── retry loop (max 3 attempts) ──────────────────┐
        ▼                                                                 │
Visual servo base (gripper YOLO)                                          │
   ├── converged → forward nudge 5.5 cm                                   │
   ├── depth_available → depth-camera fallback grasp ────────┐            │
   └── timeout / confirm_failed → back up + retry ───────────┼────────────┘
        │                                                    │
        ▼                                                    │
FK to find gripper position in base_link                     │
        │                                                    │
        ▼                                                    │
IK → lower arm to handle height                              │
        │                                                    │
        ▼                                                    │
Close gripper → read actual position ◄───────────────────────┘
        │
        ▼
Lift arm to 25 cm
        │
        ▼
Grasp confirmed? (gripper stopped short of fully closing)
   ├── yes → publish /grasp/proceed True → success
   └── no  → open gripper, back up 10 cm, retry
```

---

## Launching

### Full pipeline (perception + grasp)

```bash
ros2 launch mirte_perception grasp.launch.py
```

This starts both `perception_node` and `grasp_node`. Use this for a complete grasp run.

Optional overrides:

```bash
ros2 launch mirte_perception grasp.launch.py \
  model1_path:=/path/to/handles_model.pt \
  gripper_model_path:=/path/to/gripper_model.pt \
  confidence_threshold:=0.6 \
  handle_real_width_m:=0.03
```

### Perception only (detection and RViz visualisation)

```bash
ros2 launch mirte_perception perception.launch.py
```

Starts only `perception_node`. Useful for tuning detection or visualising in RViz without the robot moving. Also exposes `fixed_frame` as an overridable argument.

### Triggering a grasp

Once both nodes are running and a handle is visible to the front camera, call the service:

```bash
ros2 service call /grasp_handle std_srvs/srv/Trigger {}
```

The call blocks until the grasp succeeds or all retries are exhausted, then prints the result message.

### Viewing debug output in RViz

Add these topics in RViz:

| Display type | Topic |
|---|---|
| Image | `/perception/annotated_image` — front camera with detections |
| MarkerArray | `/perception/object_markers` — 3D handle positions |
| Image | `/gripper_camera/annotated_image` — servo view with centroid and target |

---

## Key calibration constants (in `grasp_node.py`)

These values were found empirically and may need re-tuning if the robot is modified:

| Constant | Value | Meaning |
|---|---|---|
| `_HOVER_JOINTS` | `[0.0, -0.433, -0.892, -1.571]` | Joint angles for the camera-down hover pose |
| `_GRASP_Z_M` | `0.06 m` | Height at which the gripper closes (handle height above floor) |
| `_APPROACH_Z_M` | `0.15 m` | Lift height after grasping |
| `_CALIB_X_M / _CALIB_Y_M` | `0.03 / 0.02 m` | Camera-to-jaw alignment offsets |
| `_SERVO_FORWARD_OFFSET_M` | `0.055 m` | Forward nudge after servo convergence |
| `_SERVO_TARGET_Y_FRAC` | `0.85` | Vertical target in gripper camera frame (85% down) |
| `_GRASP_DETECT_THRESHOLD` | `0.08 rad` | Gripper must stop this far short of fully closed to confirm a grasp |
