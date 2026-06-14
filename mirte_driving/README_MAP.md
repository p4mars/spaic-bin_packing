# README_MAP — Mapping & Localisation (`mirte_driving`)

Mapping half of `mirte_driving`. This document explains, from the ground up,
**how the robot builds a map of an unknown room and tracks where it is inside
it**, and **which parts are custom project code versus standard software that was
already available**. The navigation half (driving, obstacle avoidance, costmaps,
inflation) is in **README_NAV.md**.

The map-building algorithm itself is not custom code — it is `slam_toolbox`, a
standard, widely-used package, installed and configured here. The only custom node
on the mapping side is a small lidar pre-filter, `scan_filter`, written for this
project specifically because the arm swivels into the lidar's scan plane on the
carry leg (§4). The full "who made what" table is in §7.

---

## 0. ROS 2 vocabulary (read this if any word below is fuzzy)

ROS 2 is the framework the robot's software runs in. Five words appear constantly:

- **Node** — *one running program* that does one job. `scan_filter` is a node;
  `slam_toolbox` is a node. A robot is many nodes running at once.
- **Topic** — *a named channel nodes use to send messages*, like a radio
  frequency. One node **publishes** (writes) to it; any number **subscribe**
  (listen). Names start with `/`, e.g. `/scan`. Each topic carries one fixed
  **message type**, e.g. `sensor_msgs/LaserScan`.
- **Package** — *a folder of code that ships as one unit.* `slam_toolbox` is an
  installed package; `mirte_driving` is this project's package.
- **Node vs. executable vs. package:** a package can ship several **executables**
  (runnable programs), and each executable, when launched, becomes a running
  **node**. `slam_toolbox` (package) ships several executables; only one is run
  here (§4).
- **Transform / TF** — the one people trip over, so in full:

### What "TF" / "a transform" actually is

The robot has many **coordinate frames** — reference points, each with its own
idea of "where is (0,0) and which way is forward": the lidar has one (`laser`),
the body has one (`base_link`), the map has one (`map`). A **transform** answers
"where is frame X relative to frame Y, and how is it rotated?" — a position offset
plus a rotation. **TF** is the ROS 2 system that tracks all of them so any node can
ask "this point the lidar measured — where is it on the map?"

Why it matters for mapping: the lidar reports distances *in its own frame*. To draw
those onto one map, the software chains transforms:

```
   map  →  odom  →  base_link  →  laser
```

Read left-to-right: "the map contains an `odom` frame; inside it sits the body
`base_link`; bolted to the body is the `laser`." Follow the whole chain and a lidar
hit becomes a point on the map. **If any link is missing, mapping silently fails**
("dropping message", empty map). Much of mapping is just making sure that chain
exists — and, as §2 shows, none of that chain is built by this package.

---

## 1. The problem

The robot is switched on at an **unknown position in an unknown room**, with no
prior map. Before it can drive anywhere on purpose it must, at the same time:

1. **Map** — build an "occupancy grid" (a grid of cells: free / occupied /
   unknown) of the walls and obstacles, from the lidar; and
2. **Localise** — continuously work out its own position and heading *within* that
   growing map.

Doing both at once, from scratch, is **SLAM** — **S**imultaneous **L**ocalisation
**A**nd **M**apping. That is why the "drive in a map you already have" tools
(`amcl`, `map_server`) are not used: there is no saved map and no known start pose.
SLAM produces both, live, using the `slam_toolbox` package.

---

## 2. The transform chain for mapping — and who provides each link

SLAM can only place a scan if the full chain `map → odom → base_link → laser`
exists. Each link, and **which program publishes it**:

```
 map  ──────────►  odom        published by slam_toolbox      (installed package)
 odom ──────────►  base_link   published by the robot's base controller (vendor)
 base_link ─────►  laser       published by the robot's robot_state_publisher (vendor)
```

- **`map → odom`** is the SLAM correction — `slam_toolbox` publishes it ~50×/sec.
  `slam_toolbox` is an installed third-party package (§4).
- **`odom → base_link`** comes from the robot's **base controller** (its wheel
  driver), part of the robot manufacturer's bring-up (`mirte-ros-packages`). It is
  broadcast automatically because the bring-up sets `enable_odom_tf: true`.
- **`base_link → laser`** (and `base_link → camera`) comes from
  **`robot_state_publisher`**, a standard node that reads the robot's physical
  description (the URDF — where the lidar/camera are bolted) and publishes those
  fixed offsets. Also part of the manufacturer's bring-up.

This package publishes **none** of these transforms; the SLAM engine and the
robot's own bring-up provide all three.

---

## 3. The exact mapping pipeline (hardware → published result)

Every box below is a real node or hardware item, with the exact topic/TF between
them. Tags: **[vendor]** = robot manufacturer's bring-up, **[apt]** = installed
third-party package, **[project]** = custom code in this package.

```
 ┌──────────────┐  /scan                  ┌───────────────┐ /scan_filtered  ┌────────────────────────┐
 │ LIDAR driver │ ───────────────────────►│  scan_filter   │ ───────────────►│ slam_toolbox           │
 │   [vendor]   │  sensor_msgs/LaserScan   │  [project node]│  LaserScan w/   │  sync_slam_toolbox_node │
 └──────────────┘  raw distances          └───────────────┘  self-returns    │   [apt package]        │
                                                              removed         │                        │
   odom→base_link TF  ◄── base controller [vendor] ─────────────────────────►│ matches each scan to   │
   base_link→laser TF ◄── robot_state_publisher [vendor] ────────────────────►│ the growing map        │
                                                                              └───────────┬────────────┘
                                                                                          │ publishes:
                                                                              /map (nav_msgs/OccupancyGrid)
                                                                              map→odom TF (~50 Hz)
                                                                                          │ read by (NAVIGATION):
                                                  Nav2 global costmap, static layer  ◄─────┤
                                                  zone_detector (marker → map frame) ◄─────┤
                                                  shuttle_manager ("where am I?")    ◄─────┘
```

Only **one** box on the mapping side is project code: `scan_filter`. The engine
(`slam_toolbox`) is installed; the transforms come from the robot. Everything below
the "read by" line is *navigation* consuming the mapping output (README_NAV).

---

## 4. Every node in the mapping pipeline, and exactly how it is used

### `slam_toolbox` — the SLAM engine *(INSTALLED third-party package)*
- **What it is:** the package `ros-humble-slam-toolbox`, installed with `apt`; the
  standard online-SLAM package for this ROS 2 version. None of it is custom code —
  it is configured and run as-is.
- **It ships several executables; exactly one is run here:**
  `sync_slam_toolbox_node`. The others are deliberately not used —
  `async_slam_toolbox_node` (drops scans under load, see §6; used only in a
  throwaway bench-test launch), and `localization_…` / `map_and_localization_…` /
  `lifelong_…` (those localise in a *saved* map, which does not exist here).
- **Inputs (how it is wired):** it **subscribes** to `/scan_filtered` (set via the
  `scan_topic` param) and **reads** the `odom → base_link` transform as a rough
  guess of how the robot moved between scans.
- **Outputs (what it produces):** it **publishes** `/map`
  (`nav_msgs/OccupancyGrid`, the grid of free/occupied cells) and the
  `map → odom` transform ~50×/sec — the continuously-corrected "where is the robot,
  really."
- **How it works in one sentence:** it lines each new scan up against walls it has
  already mapped ("scan matching") and nudges its estimate so the scan fits —
  correcting the drift that wheel odometry alone builds up.
- **Services it offers that are not used here:** `/slam_toolbox/save_map`,
  `serialize_map`, etc. (mapping is live; nothing is saved/reloaded).
- **Configured entirely through** `slam_params.yaml` (§5).

### `scan_filter` — the lidar cleaner built for the swivelling arm *(PROJECT node)*
- **What it is:** one small custom node, written for this project specifically to
  deal with **the arm**. It is **not** part of the `mirte_navigation` reference —
  that setup mapped straight from the raw `/scan`, and so does plain mapping on
  this robot when the arm is tucked.
- **Why it exists — the arm swivels into the lidar plane:** on the A→B carry leg
  the arm/gripper swings forward into the lidar's scan plane (~0.25–0.35 m ahead of
  the body). Those returns land *inside* the robot's own footprint, so SLAM and the
  Nav2 costmaps mark the robot as permanently in collision — Nav2 then reports
  "collision ahead" for every motion and the leg can't run. `scan_filter` deletes
  the returns closer than `min_range` and republishes the rest as `/scan_filtered`,
  which clears the swivelled arm out of the scan. The chassis alone never required
  this (the raw-scan reference, and hand-mapping with the arm down, both map fine);
  the filter earns its keep only because the arm intrudes.
- **What it does:** **subscribes** to `/scan` (`sensor_msgs/LaserScan`) with
  **BEST_EFFORT** reliability (the lidar publishes that way; a default RELIABLE
  subscriber would receive *nothing*). It **publishes** `/scan_filtered`: a copy
  where every reading closer than `min_range` becomes "infinity" (= nothing there).
  SLAM reads it via `scan_topic: /scan_filtered`.
- **The one knob:** `min_range` (a ROS *parameter*). Default `0.25` m; the mission
  raises it to **`0.40` m specifically to clear the extended arm**. **It must match
  `raytrace_min_range` in the navigation costmaps.** Note the cost: a 0.40 m cutoff
  blinds the robot to anything within 40 cm, so keep `min_range` no larger than what
  the arm actually needs.

### The transform providers — `base controller` & `robot_state_publisher` *(VENDOR)*
Covered in §2: the robot manufacturer's bring-up publishes `odom → base_link`
(base controller) and `base_link → laser` (robot_state_publisher). Listed here only
to make the node inventory complete — both are run by the robot, not this package.

### `pointcloud_to_laserscan` *(INSTALLED, fallback only)*
On a unit with **no lidar**, the launch can start this `apt` node to synthesise a
fake `/scan` from the depth camera's point cloud. Off by default (a real lidar is
present).

---

## 5. `slam_params.yaml` — the SLAM settings *(approach from the reference; tuning is project)*

This file is the list of values handed to `slam_toolbox` at start-up. The SLAM
*approach* follows the `mirte_navigation` reference (same engine, `mode: mapping`,
scan-matching on); the specific tuning here is project work for this robot — finer
`resolution` (0.02 vs the reference's 0.05), faster `map_update_interval`, and the
`/scan_filtered` redirect (the reference read the raw `/scan`). The ones that
matter:

| Setting | Value | Plain-English reason |
|---|---|---|
| `mode` | `mapping` | Build a brand-new map (no saved map to load). |
| `scan_topic` | `/scan_filtered` | Read the cleaned scan from `scan_filter` (a project change; the reference read the raw `/scan`). |
| `use_scan_matching` | `true` | **The crucial one.** Mecanum-wheel odometry drifts badly, worst when spinning in place; scan-matching pulls the estimate back onto the real walls. This is why every in-place spin in the mission is deliberately slow (≤0.3–0.4 rad/s): spin too fast and the matcher can't keep up, so rotated, smeared copies of the room get stamped into the map. |
| `resolution` | `0.02` m/cell | Map detail; the value proven on this robot's computer. |
| `map_update_interval` | `1.0` s | Redraw the grid once a second. |
| `transform_publish_period` | `0.02` s (50 Hz) | Emit the `map→odom` correction often, so the estimate stays smooth. |
| `transform_timeout` | `0.5` s | Tolerate brief scan/TF delays under load. |
| `minimum_travel_distance` / `_heading` | `0.05` m / `0.1` rad | How far to move before adding a new scan to the records. |
| `throttle_scans` | `1` | **Process every scan** — never skip. Skipped scans during a spin lose the matcher's lock. |

**CPU contingency** (only if `slam_toolbox` *continuously* logs "queue is full"
*and* the map visibly lags): first raise `resolution` to `0.03`, then
`map_update_interval` to `2.0`. Do not switch to the async node or set
`throttle_scans: 2` — both drop scans during spins and degrade the map.

---

## 6. Why the **sync** slam_toolbox node

`slam_toolbox` ships an **async** executable and a **sync** one; the **sync** one
is used here. The async one, when the computer is busy, drops scans to keep up —
and the busiest moments are the in-place spins, which is exactly when a dropped
scan loses the matcher's lock and stamps a rotated, mis-aligned copy of the room
into the map. The sync node processes every scan in order. The CPU headroom for it
exists because the marker detector runs on the laptop and Nav2 is bundled into one
process (see README_NAV).

---

## 7. Mapping provenance — who made each piece

Four tiers:

| Piece | Type | Tier | Why it's used |
|---|---|---|---|
| `slam_toolbox` (`sync_slam_toolbox_node`) | node, installed package | **Third-party (apt)** | The standard SLAM engine; a custom one would be worse |
| `base controller`, `robot_state_publisher`, lidar/camera drivers | nodes | **Robot vendor** (`mirte-ros-packages`, preinstalled) | The robot's own driver layer — consumed (`/scan`, transforms, camera), never modified |
| `pointcloud_to_laserscan` | node | **Third-party (apt)** | Optional no-lidar fallback |
| `slam_params.yaml` | config | **Project tuning**; SLAM approach from the `mirte_navigation` reference | Reference set the approach (mapping, scan-matching); resolution + the `/scan_filtered` redirect are project tuning |
| `scan_filter.py` | small node | **Project code** | Built specifically for the swivelling arm: clears the arm out of the lidar scan on the carry leg (the reference read the raw `/scan`, which works only when the arm is tucked) |

In short: the SLAM algorithm is the standard `slam_toolbox`, run with the
`mirte_navigation` reference approach but project tuning (and a project
`scan_filter` the reference didn't have); the transforms come from the robot's own
driver layer; the only custom mapping code is `scan_filter`, which removes
self-returns to produce `/scan_filtered`.

> **Where the inflation radius lives — it is NOT mapping.** Inflation is a
> **navigation** parameter, not a mapping one. Mapping (SLAM) produces the *raw*
> map: cells free/occupied/unknown, full stop. **Inflation** grows obstacles
> outward by a safety margin so the planner keeps the robot's body clear — it lives
> in `exploration_nav2_params.yaml` and is changed per-leg by `shuttle_manager.py`.
> See **README_NAV.md §"Costmaps & inflation."** Short version: *map = mapping;
> costmap/inflation = navigation.*

---

## 8. How to check mapping is working (on its own)

```bash
# On the robot, with the stack running:
ros2 run tf2_ros tf2_echo map base_link   # must print real, changing numbers
ros2 topic echo /scan_filtered --once --field range_min   # filter is alive
ros2 topic hz /map                        # map is updating (~1 Hz)
```
In RViz (Fixed Frame = `map`): a **Map** display on `/map` should fill in while
driving; a **LaserScan** on `/scan_filtered` should land on the walls, not on the
robot. In the mission, the `WAIT_SLAM` step blocks until `map → base_link` exists,
then logs `SLAM/TF ready`.

**Common failure → cause:**
- "Waiting for SLAM (map→base_link)…" forever → no `/scan` (lidar/`scan_filter`
  down) **or** no `odom → base_link` (the robot's base driver isn't up). Check
  `ros2 run tf2_ros tf2_echo odom base_link`.
- Map smears / rotated copies → spinning too fast for the matcher, or the async
  node is in use.
- Robot frozen, "collision ahead" everywhere → self-hits inside the body; raise
  `scan_min_range`.
- `TF_OLD_DATA` spam → two programs publishing one transform; check that nothing
  outside this package is also publishing `odom → base_link`.
