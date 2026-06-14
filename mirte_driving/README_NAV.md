# README_NAV — Navigation & the A↔B Shuttle (`mirte_driving_3`)

Navigation half of `mirte_driving_3`. This document explains **how the robot
decides where to go, plans a path that won't hit anything, and drives it** — the
marker (ArUco) detection, the Nav2 navigation stack, the mission logic, and the
camera fine-alignment — and **which parts are custom project code versus standard
software that was already available**. It builds on the mapping half in
**README_MAP.md** (read that first: navigation is meaningless without the map and
the position estimate that mapping produces).

The path-planning and path-following engine is not custom code — it is **Nav2**, a
large standard collection of packages, installed and configured here. The custom
navigation code is two nodes — `zone_detector` (camera → goal poses) and
`shuttle_manager` (the mission logic) — plus the configuration, the behaviour tree,
and the launch files. The full "who made what" table is in §2.

---

## 0. ROS 2 vocabulary specific to navigation

The four core words — **node** (a running program), **topic** (a named channel
nodes send messages on), **package** (a folder of shippable code), and
**transform / TF** (where one coordinate frame sits relative to another) — are all
defined in plain English in **README_MAP.md §0**. Three more matter here:

- **Action** — like a topic message, but for a *long-running request with a
  result*. "Navigate to this pose" is an **action** (`NavigateToPose`): a goal is
  sent, progress updates come back, and finally a success/failure. This is how the
  mission asks Nav2 to drive somewhere.
- **Costmap** — a grid, like the map, but each cell holds a *cost of driving there*
  instead of just free/occupied. High cost near obstacles, low in the open; the
  planner prefers low-cost cells. **It is a navigation structure, built from the
  map — not the map itself** (see §7).
- **Nav2** — the standard ROS 2 **navigation stack**: a bundle of packages that,
  given a goal and a costmap, plans and drives a route. Configured here, not
  written here.

---

## 1. The mission, in one paragraph

Once the map exists, the robot **spins in place to find** two ArUco marker zones —
**A** (a single marker) and **B** (the midpoint between a pair of markers) —
driving to a new spot and spinning again if one spin doesn't reveal both. Then it
**shuttles A→B→A→B…** for a configurable number of round trips. Each leg: Nav2
drives to a stand-off a metre in front of the zone, a **camera servo** nudges the
robot to sit *exactly* in front of the marker, then that zone's task runs (a grab
at A / a precise dock at B / just an arm gesture, depending on flags). Static
**and moving** obstacles are avoided the whole time.

---

## 2. Navigation provenance — who made each piece

Four tiers:

| Piece | Type | Tier | Why it's used |
|---|---|---|---|
| **Nav2** servers (planner / controller / behaviors / bt_navigator / lifecycle_manager) | nodes, installed packages | **Third-party (apt)** | The standard navigation stack; a custom planner/controller would be worse |
| `rclcpp_components` (the container), `nav2_costmap_2d`, `tf2_ros` | nodes / libraries, installed | **Third-party (apt)** | Standard ROS 2 / Nav2 building blocks |
| base controller, camera driver | nodes | **Robot vendor** (`mirte-ros-packages`, preinstalled) | The robot's own driver layer — drives the wheels, streams the camera; consumed, never modified |
| `exploration_nav2_params.yaml` | config (Nav2 tuning) | **Reference config** (`mirte_navigation`), tuned for this robot | Proven Nav2 tuning baseline |
| `nav2_minimal_tree.xml` | config (behaviour tree) | **Reference config**, adapted | Defines what Nav2 does on success and on failure |
| `zone_detector.py` | node | **Project code** | Turns the camera image into goal poses (no off-the-shelf node does this for these markers) |
| `shuttle_manager.py` | node | **Project code** | The mission logic — search, shuttle, camera align, hand-offs — that Nav2 alone can't provide |
| `shuttle.launch.py`, `mission.launch.py`, `detector.launch.py` | launch files | **Project code** | Orchestrate the whole stack |
| `fastdds_udp_only.xml` | config (network profile) | **Project code** | A UDP-only DDS profile to avoid a shared-memory clash |

In short: the navigation **engine** (Nav2) and the **driver layer** are standard,
installed software; the Nav2 **tuning and behaviour tree** come from the
`mirte_navigation` reference and were adapted for this robot; the custom navigation
**code** is `zone_detector`, `shuttle_manager`, and the launch files.

### The Nav2 sub-packages (installed) and what each node does

"Nav2" is not one program — it is several nodes, each from its own installed
package, all loaded into **one shared process** (see §5). What each one does:

| Node (package, all `apt`) | Its job | Plugin/algorithm selected here |
|---|---|---|
| `bt_navigator` (`nav2_bt_navigator`) | Orchestrator. Exposes the `NavigateToPose` **action** and runs the behaviour-tree file to decide *what to do, and what to do on failure* | runs `nav2_minimal_tree.xml` |
| `planner_server` (`nav2_planner` + `nav2_navfn_planner`) | **Global planner**: finds a full path to the goal across the costmap | `GridBased` = `NavfnPlanner` (**Dijkstra**, `use_astar:false`) |
| `controller_server` (`nav2_controller` + `nav2_regulated_pure_pursuit_controller`) | **Controller**: turns the planned path into wheel-velocity commands; also holds the two costmaps | `FollowPath` = `RegulatedPurePursuitController`; `SimpleProgressChecker`; `SimpleGoalChecker` |
| `behavior_server` (`nav2_behaviors`) | **Recovery moves** used when a goal gets stuck | `spin`, `backup`, `wait` (only backup + wait are used by the tree) |
| `lifecycle_manager` (`nav2_lifecycle_manager`) | Starts up, configures, and supervises the Nav2 nodes in the right order | manages the four servers above |
| costmap layers (`nav2_costmap_2d`) | Build the costmaps inside the planner/controller | `StaticLayer`, `ObstacleLayer`, `InflationLayer` (§6–7) |

---

## 3. The exact navigation pipeline (hardware → motion)

Tags: **[vendor]** = robot manufacturer's bring-up, **[apt]** = installed package,
**[project]** = custom code, **[map]** = comes from the mapping side (README_MAP).

```
 camera [vendor] ─► zone_detector [project] ─► /zone_a_pose, /zone_b_pose ──┐  (WHERE to go)
 /map + map→odom→base_link TF [map] ────────────────────────────────────────┤  (WHERE am I / what's solid)
 /scan_filtered [map] ─► costmap obstacle layer [apt] ───────────────────────┤  (what's in the way, incl. moving)
 base velocity /…/odom [vendor] ─► controller odom smoother [apt] ───────────┘  (how fast am I going)
                              │
                              ▼
   shuttle_manager [project] ──NavigateToPose action──► bt_navigator [apt]
                                                          │ runs the behaviour tree (replans 2×/s)
                              ┌───────────────────────────┼─────────────────────────┐
                              ▼                           ▼                           ▼
                     planner_server [apt]         controller_server [apt]      behavior_server [apt]
                     (NavFn: global path)         (RPP: follow the path)       (back-up / wait)
                              │                           │
                              └──────── path ────────────┘
                                                          ▼
                            velocity command (Twist) on cmd_vel ─► robot's base controller [vendor] ─► WHEELS

   shuttle_manager [project] ALSO drives the wheels directly (Twist) for:
   the SEARCH spin, the A/B camera fine-alignment, and the 180° turn at B.
   ── EXACTLY ONE thing commands the wheels at any instant ──
```

A **Twist** is the standard message type for "drive like this": a linear speed and
a turning speed. The only custom nodes in this pipeline are `zone_detector` and
`shuttle_manager`; everything else is installed Nav2, the mapping output, or the
robot's own driver.

---

## 4. The two ways the wheels get driven

Two sources of wheel commands, only ever one active at a time:

1. **Nav2 (during legs / wander hops).** `shuttle_manager` sends a `NavigateToPose`
   goal; Nav2 plans and follows, sending velocity commands. **All obstacle
   avoidance happens here.**
2. **`shuttle_manager`'s own simple servos (direct velocity commands)**, for three
   things Nav2 can't do precisely enough:
   - **SEARCH spin** — rotate slowly in place to look for markers.
   - **Camera fine-alignment** at A and B — Nav2 is "happy" once it's within
     ±0.25 m / ±0.12 rad of the goal, which is *not* "exactly in front of the
     marker." The servo reads the live marker pose (position *and* which way the
     tag faces) and slides the (sideways-capable, "mecanum") base onto the line in
     front of the tag, to ~5 cm / ~6°.
   - **180° turn at B** before the return leg.

The hand-over rule is strict: when a wander is cancelled, the code waits for Nav2
to confirm the goal is fully finished before the spin resumes, and any late
messages from a replaced goal are ignored using a goal counter.

---

## 5. Why all the Nav2 nodes run in ONE process

All the Nav2 nodes are loaded into a single shared process (a
`component_container_isolated`, from `rclcpp_components`). The robot's onboard
computer is **communications-bound** — its messaging bus (DDS, used by nodes to
find each other) saturates before its CPU does. As five separate processes, the
Nav2 nodes plus the robot's own system processes create a discovery storm that can
take ~25 s to settle, and the start-up manager's calls then time out and abort.
Bundled into one process, those start-up calls happen *inside* the process (no
network discovery, can't time out) and Nav2 looks like a *single* participant on
the bus. The launch also staggers start-up (Nav2 begins 35 s in) so discovery
settles and the map exists first.

---

## 6. Obstacle avoidance — static **and** moving

The heart of navigation. Two costmaps work together (both built by
`nav2_costmap_2d`, configured in `exploration_nav2_params.yaml`):

- **Global costmap** (in the `map` frame) — used to plan the whole route. Three
  **layers** stacked on top of each other:
  - **static layer** — the SLAM map (the walls). *This is the only place the
    mapping output enters navigation.*
  - **obstacle layer** — this instant's lidar (`/scan_filtered`), so newly-seen
    obstacles appear and freed space is cleared.
  - **inflation layer** — see §7.
  Set to `allow_unknown: true` so it can route through not-yet-mapped space while
  the map is still being built.
- **Local costmap** — the **moving-obstacle catcher**: a small rolling **5×5 m**
  window around the robot (in the `odom` frame, so it keeps working even if the map
  correction lags), refreshed **4×/sec** straight from the lidar. A person stepping
  into the path is marked within ~250 ms.
- **Controller collision check:** RPP projects the robot's **footprint** (its
  outline, sized to include the arm) ~2 s ahead along the intended motion and
  refuses to drive if that would hit something; the global plan (re-run 2×/sec)
  then routes around it.
- **The behaviour tree** (`nav2_minimal_tree.xml`): replan 2×/sec; on failure
  **back up 0.15 m + wait**, retry up to 6 times. It deliberately **never clears
  the costmap** — clearing would erase every known obstacle, so the robot would
  replan on a blank grid and drive through things it had been avoiding. Obstacles
  that genuinely move away still vanish on their own, because lidar rays passing
  through their old cells clear them.

---

## 7. Costmaps & inflation — **this is NAVIGATION, not mapping**

- **Mapping (SLAM) produces the raw `/map`:** each cell is just free, occupied, or
  unknown. That's all a map is. Mapping has no concept of "keep a safety margin."
- **Navigation turns that map into a *costmap*:** it copies the walls in (the
  static layer), adds live lidar (the obstacle layer), and then **inflates** —
  grows every obstacle outward by a safety margin and paints a fading "cost halo"
  around it, so the planner naturally keeps the *whole width of the robot* away
  from walls instead of clipping its corner. **Inflation only exists in the
  costmap, which is a navigation structure.**

| What | Where it lives | Notes |
|---|---|---|
| `inflation_radius` (0.30 m), `cost_scaling_factor` (3.5) | `exploration_nav2_params.yaml`, in the `inflation_layer` of each costmap | The static halo size/steepness. **Navigation.** |
| The robot's `footprint` (0.32 m front, ±0.25 m side) | same file, in each costmap | Sized to cover the chassis **+ the box-carry arm**. |
| **Per-leg** inflation: 0.45 m while carrying (A→B), 0.28 m empty (B→A) | set live by **`shuttle_manager.py`** (`_set_inflation`, params `inflation_carry` / `inflation_empty`) | The mission widens the margin on the carry leg, where the arm sticks out, by calling Nav2's parameter service mid-mission. **Navigation.** |

`shuttle_manager` also raises the cruise **speed** live once both zones are found
(`_set_speed`). None of this touches `slam_params.yaml` — mapping's config is
untouched by any of it. In one line: **`/map` is mapping; the costmap and its
inflation are navigation.**

---

## 8. The mission decision logic (`shuttle_manager`)

```
WAIT_SLAM ─ map→base_link exists? ─► SEARCH ─ both zones seen? ─► SHUTTLE ─ legs done ─► DONE
   │ (60 s timeout → abort)            │                            │
   └ arm → upright (held)              │ spin one revolution;       │ per leg:
                                       │ if not both seen → drive    │  ① face the next zone (gentle turn)
                                       │ to a reachable on-map       │  ② set per-leg inflation + cruise speed
                                       │ vantage and spin again      │  ③ Nav2 drive to the 1 m stand-off
                                                                     │  ④ camera fine-align (exactly in front)
                                                                     │  ⑤ zone task: grasp@A / dock@B / arm gesture
                                                                     │  ⑥ (at B) 180° turn, then next leg
```

**Search/wander detail:** wander targets are checked to be **on the costmap**
(`_in_map` — an off-map goal makes the planner abort) and in an **open cell**
(`_has_clearance`), across a fan of headings. A clear straight line of sight is
deliberately *not* required (that over-strict check makes the robot decide it is
"boxed in" and spin forever) — Nav2's planner does the real obstacle avoidance on
the way to the vantage point.

**Mode flags** (set at launch; the navigation-only test turns everything off except
the aligns):

| flag | default (mission) | effect |
|---|---|---|
| `align_at_a`, `align_at_b` | true | the camera fine-alignment servos (part of navigation) |
| `arm_mimic` | true | box carry/drop arm gestures (no real grasp) |
| `dock_at_b` | false | spawn the `mirte_placement` package at B for the precise dock + box place |
| `grasp_at_a` | false | hand A off to the laptop's `mirte_perception` grasp |
| `turn_at_b` | true | spin 180° after the B align |
| `round_trips` | 3 | number of A→B→A round trips |

---

## 9. `exploration_nav2_params.yaml` — the settings that matter

Values from the `mirte_navigation` reference, tuned for this robot.

| Parameter | Value | Why |
|---|---|---|
| planner `GridBased` | NavFn, `use_astar: false`, `allow_unknown: true` | Dijkstra; plan through unknown space while the map grows |
| `controller_frequency` | `3.0` Hz | 10/5 Hz are missed on this computer; a controller that misses its deadline is worse than a slower steady one |
| RPP `desired_linear_vel` | `0.22` m/s (search) → `0.32` (shuttle, set live) | Cautious while searching; cruise once both zones are known |
| RPP `use_collision_detection` | `true` | React to **moving** obstacles, not just mapped ones |
| RPP `max_allowed_time_to_collision…` | `2.0` s | ≈0.5 m + footprint of forward look — stop *before* a person on the path |
| RPP `lookahead_dist` | `0.5` m (0.25–1.0 scaled) | Tight path following, less corner-cutting toward obstacles |
| RPP `use_rotate_to_heading` | `true` | A stand-off goal can be "stay put but face this way"; without this the controller can't achieve the heading and times out forever |
| RPP `rotate_to_heading_angular_vel` | `0.4` rad/s | **Gentle** — fast in-place spins outrun the scan-matcher and the position estimate diverges (README_MAP §6) |
| `use_cost_regulated_linear_velocity_scaling` | `true` | Automatically slow down through high-cost cells near obstacles |
| progress checker | `0.05` m / `45` s | Lenient — the robot moves in fits under bus load; don't abort a slow-but-moving leg |
| goal checker | `xy 0.25` m, `yaw 0.12` rad | Tight heading so it ends squarely facing the marker (forward camera); the align servo then refines |
| local costmap | rolling `5×5` m, `4` Hz, `transform_tolerance 1.0` | Catch moving obstacles early; tolerate bursty position updates |
| footprint | `0.32` front, `±0.25` side | Covers chassis **+ the box-carry arm** |
| inflation | `0.30` m (live: `0.45` carry / `0.28` empty) | Path keeps the whole robot clear; wider on the A→B carry leg where the arm is out (§7) |
| `transform_tolerance` (both costmaps) | `1.0` s | The real robot's SLAM publishes the `map→odom` correction in bursts under load; the default 0.3 s would drop every scan |

`shuttle_manager` overrides two of these **live** through Nav2's parameter service:
the inflation per leg (`_set_inflation`) and the cruise speed once both zones are
found (`_set_speed`).

---

## 10. Per-unit settings handled by launch args (not code edits)

These are **flags**, never source edits:

| Arg | What it controls |
|---|---|
| `cmd_vel_topic` / `odom_topic` | sim drives `/cmd_vel`+`/odom`; the real robot uses `/mirte_base_controller/{cmd_vel,odom}`. A silent odom topic pins the controller's speed-scaled lookahead at minimum (twitchy following) |
| `run_zone_detector` | detect markers on the robot (true) vs. offload to the laptop (false) to free the robot's messaging bus |
| `udp_only` | force UDP-only networking (scoped to the launch) to dodge a clash with the boot service over shared-memory |
| `scan_min_range` | raise it when the arm sits in the lidar's plane and trips "collision ahead" (a mapping/scan setting — README_MAP §4) |
| `use_depth_scan` | synthesise a fake `/scan` from the depth camera on a unit that has no lidar |

`odom → base_link` and the sensor-mount transforms are **not** published by this
package — the robot's own bring-up provides them (README_MAP §2).

---

## 11. How to run & verify navigation

**Navigation-only test (pure A↔B shuttle, wheels + camera align, nothing else):**
```bash
# robot
ros2 launch mirte_driving_3 mission.launch.py \
  run_zone_detector:=false dock_at_b:=false grasp_at_a:=false arm_mimic:=false \
  round_trips:=2
# laptop (zone detection)
ros2 launch mirte_driving_3 detector.launch.py   # same network + ROS_DOMAIN_ID
```
**Full merged mission** = the same command with the flags at their defaults
(`dock_at_b`/`grasp_at_a` engage the manipulation packages at the right moments).

**Expected log arc:** `SLAM/TF ready` → `Spinning to find zones (have […])` →
(wander) → `Both zones found — starting shuttle` → `Cruise speed → 0.32` →
`Leg 1/N → Zone A` → `Aligned in front of Zone A` → … → `Leg 2 → Zone B` →
`Aligned in front of Zone B` → `turning 180°` → return.

**Quick checks:**
```bash
ros2 topic echo /zone_a_pose --once          # detector producing goals?
ros2 topic info /mirte_base_controller/cmd_vel --verbose   # who's driving (1 at a time)
ros2 run tf2_ros tf2_echo map base_link      # localisation alive (README_MAP)
```

**Common failure → cause:**
- Spins forever `have ['A']` → both B markers not seen; move the robot / check the
  detector and marker ids (A=100, B=101/102, DICT_4X4_250, 8 cm).
- `failed to create plan` between zones → carry inflation too wide for a tight gap;
  lower `inflation_carry` (§7).
- Robot frozen, "collision ahead" everywhere → arm/self in the lidar (raise
  `scan_min_range`) — a mapping/scan issue, see README_MAP §8.
- Distance to A consistently wrong → `zone_marker_size` ≠ the printed tag size (all
  camera distances scale with it).
