# Oral Exam — Navigation Half — Battle Plan

Everything to know by heart, what to show and in what order, what to say you'd
improve, and the exact edits you may be asked to make.

Your part = **navigation**: `zone_detector.py` and `shuttle_manager.py` are *yours*
to defend and edit. Mapping (`scan_filter.py`, `slam_params.yaml`) is your
teammate's — explain it at a high level, then say "that's the mapping half."

> **Package cleanliness (mirte_driving):** clean. Tidy ROS 2 layout (`mirte_driving/`
> nodes, `params/`, `trees/`, `launch/`, `config/`, `resource/mirte_driving`, three
> READMEs), no stray/editor junk, no `build/`/`install/` inside it, zero
> `mirte_driving_3` leftovers. Two trivial blemishes if you want it perfect: local
> `__pycache__/` folders (add a `.gitignore` with `__pycache__/` + `*.pyc`), and the
> `license` field is still `TODO: License declaration` in `package.xml` + `setup.py`.

---

## 0. What the exam actually is (from two students who took it)

- You **explain your part**: which nodes, how they're linked, and **the link between
  the robot's hardware and the published result**.
- You **draw** how your code connects to all sensors *and* the other members' code.
- You get **questions on your own part** specifically.
- You're then asked to **change the code live** — usually framed as *"what would you
  do with more time?"* → whatever you name becomes the task.
- They give **too little time**. ⇒ **Name something simple you can actually finish**
  (§6).
- Examiners also value a **tidy repo** — don't show a code dump.

Real requests they got: *"make the AprilTags stay in RViz forever instead of
disappearing when the camera turns away"* and *"fix the depth-camera fusion"*
(impossible in the time — don't get cornered into something like that).

---

## 1. The 60-second pitch (memorise the opening)

> "I did the **navigation** half. The robot is dropped in an unknown arena. My two
> nodes are **`zone_detector`** and **`shuttle_manager`**. `zone_detector` is the
> eyes — it takes the camera image, finds the ArUco markers with OpenCV, and turns
> them into two goal poses on the map: Zone A (one marker) and Zone B (the midpoint
> of a marker pair). `shuttle_manager` is the brain — a state machine that waits for
> the map, spins to find the zones, then shuttles A↔B. For each leg it sends a goal
> to **Nav2** (the standard navigation stack — I configured it, didn't write it),
> which plans a path and drives the wheels while avoiding obstacles; then my own
> code does a camera fine-alignment to sit exactly in front of the marker. It also
> hands off to the other members' packages — the grasp at A and the box placement
> at B."

Then offer the diagram.

---

## 2. The diagram you must be able to draw

Practise drawing this in under two minutes. Three columns: **hardware → my nodes →
Nav2 / other members.** Arrows are topics; label them.

```
   ROBOT HARDWARE                 MY NAVIGATION CODE                 NAV2 (configured, not written)        OTHER MEMBERS
   ──────────────                 ──────────────────                 ─────────────────────────────        ─────────────

   Camera ──/camera/image_raw──►  zone_detector ──/zone_a_pose──┐
           /camera/camera_info    (OpenCV cv2.aruco)  /zone_b_pose │
                                                                  ▼
                                         shuttle_manager ──NavigateToPose (action)──►  Nav2:
                                          (the FSM / brain)                            planner + controller
   Lidar ──/scan──► scan_filter ──/scan_filtered──────────────────────────────────►  + 2 costmaps + BT
           (mapping)              │                                                        │
                                  └──► slam_toolbox ──/map, map→odom TF──► costmaps        │ Twist
                                       (mapping)            │                              ▼
   Wheel encoders ──► base controller ──/odom, odom→base_link TF ──► (localisation)   /mirte_base_controller/cmd_vel ─► WHEELS
                                                                                          ▲
                          shuttle_manager ALSO sends Twist directly here for: ───────────┘
                          search spin · camera fine-align · 180° turn

   At Zone A:  shuttle_manager ──/grasp_handle (service)──►  mirte_perception (laptop, YOLO grasp)
   At Zone B:  shuttle_manager ◄──handshake topics──►        mirte_placement (marker_navigator + box_placer)
                                (/robot_positioned, /box_placed, /robot_turned_around, …)
```

**The "hardware → published result" chain, said out loud:**
1. *Camera* → raw pixels on `/camera/image_raw`.
2. `zone_detector` runs `cv2.aruco` → marker pose in the **camera** frame, then
   **TF** → **map** frame → publishes `/zone_a_pose`, `/zone_b_pose`.
3. *Lidar* → `/scan` → (mapping) `scan_filter` → `slam_toolbox` → `/map` + the
   `map→odom` transform.
4. `shuttle_manager` takes a zone pose as a goal → Nav2 plans on the costmap → emits
   a velocity `Twist` → the *base controller* turns the *wheels*. Pixels-and-laser-in,
   wheel-motion-out.

---

## 3. What you must know by heart

**Your two nodes, one line each:**
- `zone_detector` = camera image → ArUco → two map-frame goal poses. (You wrote it.)
- `shuttle_manager` = the mission state machine driving the whole A↔B cycle. (You
  wrote it.)

**Provenance — say this exactly so you never over-claim:**
- **Wrote from scratch:** `zone_detector.py`, `shuttle_manager.py`, the launch files.
- **Configured / tuned (didn't write):** **Nav2** is a standard installed package;
  `exploration_nav2_params.yaml` is my tuning, modelled on the course's
  `minimal_nav2_params.yaml`.
- **Adapted (not mine originally):** `nav2_minimal_tree.xml` is the reference's
  behaviour tree, modified (removed costmap-clearing).
- **Used, not written:** OpenCV `cv2.aruco` is a *library* I call inside my node —
  not a ready-made detector node like `aruco_ros`.
- **Not mine (mapping teammate):** `scan_filter.py`, `slam_params.yaml`,
  `slam_toolbox`.
- **Robot vendor:** base controller, `robot_state_publisher`, drivers.

**Key numbers:** ArUco **`DICT_4X4_250`**, marker size **0.08 m**; Zone A = id
**100**, Zone B = midpoint of ids **101 (left) + 102 (right)**; `round_trips` 3;
cruise 0.32 m/s; inflation 0.30 m (0.45 carrying / 0.28 empty).

**The dependency to acknowledge:** `zone_detector` needs SLAM's `map→odom` transform
to place markers in the map. *"If the mapping side isn't up, my TF lookup fails and I
publish nothing"* — shows you understand the coupling.

---

## 4. Which code to show, and in what order

Walk it as a pipeline: **wire-up → where to go → the brain → how it drives → failure
handling.** Read each file top-to-bottom (the header comments were written for this
order).

1. **`launch/shuttle.launch.py`** — read only the header (node-graph + startup
   timeline). *"This is how the stack is assembled."* `mission.launch.py` is the
   real-robot wrapper.
2. **`zone_detector.py`** — the input (§5a).
3. **`shuttle_manager.py`** — the brain; spend the most time (§5b).
4. **`params/exploration_nav2_params.yaml`** — how Nav2 plans/drives/avoids.
5. **`trees/nav2_minimal_tree.xml`** — failure handling (say "adapted").

**Do not** open `scan_filter.py` or `slam_params.yaml` as "your" code.

---

## 5a. Deep cheat-sheet — `zone_detector.py`

**Pipeline:** camera pixels → `cv2.aruco.detectMarkers` → `estimatePoseSingleMarkers`
(pose in camera frame, using marker size + intrinsics) → **TF** to map → EMA
smoothing + outlier rejection → publish `/zone_a_pose`, `/zone_b_pose`.

**Must explain:**
- **What CV it uses:** OpenCV's `cv2.aruco` (a *library*, in-process), `cv_bridge`
  (ROS image → OpenCV), NumPy for maths.
- **Why intrinsics (`/camera/camera_info`):** `fx, fy, cx, cy` convert "a marker of
  known real size at these pixels" into a metric 3D position.
- **Why Zone B is two markers:** the stand carries a pair; I publish the **midpoint**
  so the robot drives to the stand; the precision dock does cm-level work between the
  tags.
- **Two clever bits:**
  - **EMA + outlier rejection** (`_accept_xy`, ~L419): a reading >0.6 m from the
    estimate is rejected unless it persists; else smoothed. Stops the goal jumping
    while spinning / from SLAM drift.
  - **Facing-normal orientation** (`_update_normal`, ~L440): single-tag ArUco
    orientation flips frame-to-frame, so I publish the tag's smoothed *ground-plane
    facing direction* instead of the raw quaternion.
- **Why it survives the camera looking away:** each marker's smoothed pose is cached
  in `self._track` and **re-published at 5 Hz from a timer** (`_publish_zones`,
  ~L471), not only on detection. (This is the answer to §7.1.)

## 5b. Deep cheat-sheet — `shuttle_manager.py`

**A state machine on a timer.** `_tick()` (~L528) runs a few Hz and switches on
`self._state`:
- **`WAIT_SLAM`** (~L540): block until `map→base_link` exists; arm raised. 60 s
  timeout → abort.
- **`SEARCH`** (~L559): spin to find both zones; if not both seen, drive to a new
  on-map vantage and spin again. → `SHUTTLE`.
- **`SHUTTLE`** (~L635): per leg — face the zone, set per-leg inflation + speed, send
  the Nav2 goal to the stand-off, camera fine-align, run the zone task (grasp@A /
  dock@B / arm gesture), 180° turn at B, advance. After `round_trips` → `DONE`.

**Two ways it drives the wheels (know cold):**
1. **Via Nav2** — `_send_goal` (~L808) sends a `NavigateToPose` action; Nav2 plans +
   follows + avoids. Used for legs and wander hops.
2. **Directly (own `Twist`)** — for the **search spin**, **camera fine-align**
   (`_align_step`, ~L1130), and the **180° turn**. Nav2's ±0.25 m tolerance isn't
   precise enough to sit in front of a marker. *Exactly one* source commands the
   wheels at a time.

**Live tuning:** `_set_inflation` (~L992) and `_set_speed` (~L1005) call Nav2's
parameter service mid-mission. **Hand-offs:** at A → `/grasp_handle` service; at B →
spawns `mirte_placement` and waits on handshake topics. **Knobs:** every
flag/distance/speed is a `declare_parameter` near the top (L130–290) — this is what
makes live edits easy.

---

## 6. "What I'd do with more time" — volunteer ONE (all simple, all yours)

Steer the exam here.
1. **Visualise the zones in RViz as persistent markers** (§7.1) — *best*: matches the
   "keep markers in RViz" request another group got, and you can half-answer it with
   code you already have.
2. **Publish the mission state on a topic for debugging** (§7.3) — tiny, shows you
   understand the FSM.
3. **Promote a hard-coded smoothing constant to a ROS parameter** (§7.2) — textbook
   "make it tunable."

Do **not** volunteer depth fusion, new perception, SLAM re-tuning, or "make the
mission more robust." Time-sinks or not your half.

---

## 7. Change requests — exact edits with code

Rebuild after any change:
```bash
cd ~/spatial-ai/ws && colcon build --packages-select mirte_driving --symlink-install && source install/setup.bash
```

### 7.1 Make the zones show (and stay) in RViz — most likely request

**The free answer first:** *"The pose already persists — I cache each marker and
re-publish `/zone_a_pose` at 5 Hz from a timer, so a Pose display in RViz stays put
when the camera looks away. The disappearing problem happens when a detector only
publishes while the tag is in view; mine doesn't."* If they want a visible **box**,
in `zone_detector.py`:

```python
# (1) top, with the other imports:
from visualization_msgs.msg import Marker

# (2) in __init__, next to self._pub_a / self._pub_b:
self._pub_marker = self.create_publisher(Marker, '/zone_markers', 10)

# (3) add this helper method to the class:
def _zone_marker(self, marker_id, xy, r, g, b):
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = self.get_clock().now().to_msg()
    m.ns = 'zones'
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = float(xy[0])
    m.pose.position.y = float(xy[1])
    m.pose.position.z = 0.1
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 0.15
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
    m.lifetime = Duration(seconds=0).to_msg()   # 0 = NEVER expires -> stays forever
    return m

# (4) in _publish_zones, after each pose is published:
self._pub_marker.publish(self._zone_marker(0, ta['xy'], 0.0, 1.0, 0.0))   # Zone A, green
self._pub_marker.publish(self._zone_marker(1, mid_xy, 0.0, 0.0, 1.0))     # Zone B, blue (both-B-seen block)
```
**The winning sentence:** *"The `lifetime` field is the mechanism — `0` means RViz
keeps the marker forever; a positive value is what makes markers vanish."* `Duration`
is already imported in this file. **Test:** rebuild, relaunch detector, in RViz
(Fixed Frame `map`) add a *Marker* display on `/zone_markers`; the cube appears and
stays when the camera turns away.

### 7.2 Promote a hard-coded constant to a ROS parameter

In `zone_detector.py`:
```python
# in __init__, with the other declare_parameter calls:
self._ema_alpha = float(self.declare_parameter('position_smoothing', EMA_ALPHA).value)

# in _accept_xy (~L435), replace the two EMA_ALPHA uses:
tr['xy'] = (cur[0] + self._ema_alpha * (px - cur[0]),
            cur[1] + self._ema_alpha * (py - cur[1]))
```
**Test:** `ros2 param set /zone_detector position_smoothing 0.4` (higher = snappier).
Say: *"Now it's tunable at launch or live without rebuilding."*

### 7.3 Publish the mission (FSM) state on a topic

In `shuttle_manager.py` (`String` is already imported):
```python
# in __init__, with the other publishers:
self._state_pub = self.create_publisher(String, '/shuttle_state', 10)

# at the very top of _tick(self):
self._state_pub.publish(String(data=self._state))
```
**Test:** `ros2 topic echo /shuttle_state` shows `WAIT_SLAM -> SEARCH -> SHUTTLE ->
DONE` live.

### 7.4 Tune a Nav2 parameter (speed / inflation / tolerance)

In `params/exploration_nav2_params.yaml`:
- **Clips walls** → raise `inflation_radius` (each `inflation_layer`, ~L183 & ~L243)
  from `0.30`.
- **Too slow** → raise `desired_linear_vel` in `controller_server` → `FollowPath`.
- **Stops too far out** → lower `xy_goal_tolerance` / `yaw_goal_tolerance` in
  `goal_checker`.

Live, no rebuild: `ros2 param set /controller_server FollowPath.desired_linear_vel
0.4`. Note per-leg inflation is also set in code in `shuttle_manager._set_inflation`
(~L992) via `inflation_carry`/`inflation_empty` (L289–290).

### 7.5 Change the failure behaviour (behaviour tree)

In `trees/nav2_minimal_tree.xml`:
- More retries: `number_of_retries="6"` → `"10"`.
- Add a spin recovery:
```xml
<RoundRobin name="RecoveryActions">
  <BackUp backup_dist="0.15" backup_speed="0.05"/>
  <Spin spin_dist="1.57"/>          <!-- added: rotate ~90 deg to re-find a path -->
  <Wait wait_duration="2.0"/>
</RoundRobin>
```
Say: *"`spin` is already configured in `behavior_server`, so the tree can call it —
and I deliberately keep no `ClearEntireCostmap`, so it never forgets obstacles."*

### 7.6 Change marker IDs / dictionary / size

No code edit — launch args in `mission.launch.py`: `zone_a_id`, `zone_b_left_id`,
`zone_b_right_id`, `aruco_dict`, `zone_marker_size` (defaults 100 / 101 / 102 /
`DICT_4X4_250` / 0.08).

### 7.7 Change the search spin (speed / direction)

`search_angular` is a parameter (~L161, default 0.3). Reverse direction = negate it
where the search `Twist` is built in the `SEARCH` block. Say: *"slower = better for
SLAM scan-matching; that's why it's gentle."*

---

## 8. Question bank — likely questions, crisp answers

- **"Walk me from a sensor to a wheel turning."** → the §2 chain.
- **"Is the ArUco detection your code or a package?"** → "The detection is OpenCV's
  `cv2.aruco` *library*, called inside my node; the node that turns detections into
  stable map-frame goals is mine. I didn't use a ready-made detector node."
- **"Why is Zone B two markers?"** → midpoint = drive to the stand; the precision dock
  does cm-level work between the pair; the baseline gives a stable facing direction.
- **"What if the camera loses the marker mid-leg?"** → the smoothed pose is cached and
  re-published at 5 Hz; Nav2 keeps driving to the last goal; alignment re-acquires
  when it's back in view.
- **"What if SLAM isn't running?"** → my `map<-camera` TF lookup fails, I publish
  nothing, `shuttle_manager` stays in `WAIT_SLAM`. Navigation depends on mapping.
- **"How do you avoid obstacles?"** → Nav2 costmaps: global (the SLAM map) for
  planning + a local 5x5 m costmap updated 4 Hz from the lidar for *moving*
  obstacles; the controller projects the footprint ahead and refuses to drive into
  something; the tree backs up + waits + replans.
- **"Why Nav2 and not write your own planner?"** → it's the standard, robust stack;
  my value-add is the mission logic and the configuration.
- **"Why Dijkstra (NavFn) not A*?"** → small arena, goal changes as the map grows;
  Dijkstra is robust and A*'s speed-up is irrelevant here.
- **"Why a separate alignment servo if Nav2 drives there?"** → Nav2's tolerance
  (±0.25 m / ±0.12 rad) isn't "exactly in front of the marker"; my servo closes it to
  ~5 cm / ~6 deg using the live marker pose.
- **"Why a 'facing normal' not the marker's quaternion?"** → single-tag ArUco
  orientation is ambiguous and flips; the normal is stable and is all the align servo
  needs.
- **"What's the most fragile part?"** → the onboard computer is communications-bound;
  that's why the detector can run on the laptop and Nav2 is composed into one process.
- **"What would you improve?"** → §6 (your rehearsed simple one).

---

## 9. Exam-day tactics

- **Lead with the diagram and the pitch** — you control the framing and pre-answer
  half their questions.
- **Be honest about provenance** (§3): "Nav2 is a package I configured; this node is
  mine" beats claiming everything.
- **When they say "change it":** restate the task in one sentence, say which file and
  why, then type while narrating. If it's your volunteered change (§6), you've
  rehearsed it.
- **If pushed toward something hard** (depth fusion, new perception): redirect — "the
  cleanest small improvement on my side is X" (§6).
- **Always rebuild + show it running:** `colcon build --packages-select mirte_driving
  --symlink-install`, then `ros2 topic echo` the new topic. A change you can *show*
  beats one you describe.
- **If the build breaks under pressure:** `python3 -m py_compile
  mirte_driving/<file>.py` to find the syntax error fast.
