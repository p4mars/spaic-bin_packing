# README_MAP — Mapping & Localisation (`mirte_driving_3`)

How the robot builds a map of an unknown room and keeps track of where it is inside
it. Everything else in the project (navigating, finding the markers, the arm
hand-offs) is built on top of this. The *navigation* half — driving, obstacle
avoidance, the costmaps, the inflation radius — is a separate document,
**README_NAV.md**.

> **Tell-the-professor summary (mapping):** the only mapping files *we* wrote are
> **`scan_filter.py`** (one node) and **`slam_params.yaml`** (config). The actual
> SLAM algorithm is `slam_toolbox`, an off-the-shelf package we installed and
> configured but did **not** write. See the ownership table in §7.

---

## 0. First, the ROS 2 vocabulary (read this if any word below is fuzzy)

ROS 2 is the framework the robot's software is built in. Four words appear
constantly; here is exactly what each one means in this project:

- **Node** — *one running program* that does one job. Example: `scan_filter` is a
  node; `slam_toolbox` is a node. A robot is many nodes running at once, talking
  to each other.
- **Topic** — *a named channel that nodes use to send messages*, like a radio
  frequency. One node **publishes** (writes) onto a topic; any number of nodes
  **subscribe** (listen) to it. Topic names start with `/`, e.g. `/scan`. The
  thing sent on a topic has a fixed **message type**, e.g. `sensor_msgs/LaserScan`.
- **Package** — *a folder of code that ships as one unit.* `mirte_driving_3` is
  our package. `slam_toolbox` is someone else's package we installed. A package
  can contain several nodes.
- **Transform / TF** — this is the one people trip over, so in full:

### What "TF" / "a transform" actually is

The robot has many **coordinate frames** — little reference points, each with its
own notion of "where is (0,0) and which way is forward." The lidar has one
(`laser`), the body has one (`base_link`), the map has one (`map`), and so on.

A **transform** is just the answer to "where is frame X relative to frame Y, and
how is it rotated?" — a position offset plus a rotation. **TF** (short for
"transforms") is the ROS 2 system that keeps track of all of them and lets any
node ask "given a point measured by the lidar, where is that point on the map?"

Why it matters for mapping: the lidar reports distances *in its own frame*. To
draw those hits onto a single map, the software must chain transforms together:

```
   map  →  odom  →  base_link  →  laser
```

Read left to right: "the map contains an `odom` frame; inside `odom` sits the
robot body `base_link`; bolted onto the body is the `laser`." Follow that whole
chain and a lidar hit becomes a point on the map. **If any link in the chain is
missing, mapping silently fails** (SLAM logs "dropping message" and the map never
fills in). So a big part of mapping is just making sure that chain exists.

---

## 1. The problem we are solving

The robot is switched on at an **unknown position in an unknown room**. It has no
prior map and no idea where it is. Before it can drive anywhere on purpose it must
do two things *at the same time*:

1. **Map** — build a picture (an "occupancy grid": a grid of cells marked
   free / occupied / unknown) of the walls and obstacles, using the lidar; and
2. **Localise** — continuously work out its own position and heading *within* that
   map as the map grows.

Doing both at once, from scratch, is called **SLAM** — **S**imultaneous
**L**ocalisation **A**nd **M**apping. That is why we do **not** use the more
common "drive in a map you already have" tools (`amcl` for localisation, or
`map_server` to load a saved map): we have no saved map and no known start pose.
SLAM produces both, live.

---

## 2. The transform chain for mapping, and who provides each link

SLAM can only place a scan if the full chain `map → odom → base_link → laser`
exists. Here is each link and **which program publishes it** — note that **we
publish none of them ourselves**:

```
 map  ──────────►  odom        published by slam_toolbox (this IS the SLAM output)
 odom ──────────►  base_link   published by the ROBOT'S OWN base controller
 base_link ─────►  laser       published by the ROBOT'S OWN robot_state_publisher
                                (reads the mounts from the robot's URDF model)
```

- **`map → odom`** is the SLAM correction. `slam_toolbox` publishes it ~50×/sec.
  This is the link *we are responsible for* in the sense that our config and our
  scan filtering make it accurate — but the code that emits it is `slam_toolbox`,
  not us.
- **`odom → base_link`** comes from the robot's **base controller** (its wheel
  driver), which already broadcasts it because the robot's own bring-up sets
  `enable_odom_tf: true`. We do **not** publish this.
- **`base_link → laser`** (and `base_link → camera`) comes from the robot's
  **`robot_state_publisher`**, a standard node that reads the robot's physical
  description (the URDF model — where the lidar and camera are bolted on) and
  publishes those fixed offsets as transforms. We do **not** publish this either.

> **Important history (why this README used to mention extra nodes).** Earlier we
> ran a small node (`odom_to_tf`) and two hand-typed mount transforms to provide
> `odom → base_link` and `base_link → laser/camera` ourselves. It turned out the
> robot already publishes all of those. Having **two** programs publish the *same*
> transform makes TF flip-flop between them ("`TF_OLD_DATA`" warnings) and the
> robot once drove into a wall because of it. **Those duplicates have now been
> deleted.** We rely entirely on the robot's own bring-up for `odom → base_link`
> and the sensor mounts. Mapping owns only the scan cleaning and the SLAM config.

---

## 3. The mapping data flow (hardware → published result)

```
 ┌──────────┐   /scan                    ┌─────────────┐  /scan_filtered      ┌───────────────┐
 │  LIDAR    │ ─────────────────────────►│ scan_filter │ ────────────────────►│ slam_toolbox  │
 │ (hardware)│  sensor_msgs/LaserScan     │  (OUR node) │  LaserScan w/ self-   │  (sync node,  │
 └──────────┘   raw distances            └─────────────┘  returns removed      │  not ours)    │
                                                                               │               │
   odom→base_link TF  (from the robot's base controller) ─────────────────────►│  matches each │
   base_link→laser TF (from the robot's robot_state_publisher) ────────────────►│  scan to the  │
                                                                               │  growing map  │
                                                                               └──────┬────────┘
                                                                                      │ produces:
                                                                          /map  (OccupancyGrid)
                                                                          map→odom  TF (the fix)
                                                                                      │ read by:
                                              Nav2 global costmap (static layer) ◄────┤  (navigation)
                                              zone_detector (marker → map frame)  ◄────┤  (navigation)
                                              shuttle_manager (where am I?)       ◄────┘  (navigation)
```

The two boxes that are **mapping's job**: `scan_filter` (ours) and `slam_toolbox`
(installed). Everything below the dashed "read by" line is navigation reading the
mapping output.

---

## 4. The mapping nodes, explained

### `scan_filter.py` — the lidar clean-up node *(WE wrote this)*
- **What kind of thing it is:** one node, in our package.
- **Subscribes to** the topic `/scan` (message type `sensor_msgs/LaserScan` — a
  ring of distance readings from the lidar). It listens with a setting called
  **BEST_EFFORT** reliability, because the lidar publishes that way; a node that
  insisted on guaranteed delivery (RELIABLE) would receive *nothing* and you'd
  get a silent dead pipeline.
- **Publishes** the topic `/scan_filtered` (same message type): a copy of the scan
  where every reading closer than `min_range` is replaced by "infinity" (meaning
  "nothing there").
- **Why it exists:** the lidar sits ~10 cm in front of the body and can see *the
  robot itself* — its own chassis, wheels, and (when carrying the box) the arm.
  Those very-close readings would otherwise be mapped as a solid obstacle sitting
  right on top of the robot, so SLAM could never build a clean map and navigation
  would scream "collision!" for every move. The filter deletes those self-hits.
- **The one knob:** `min_range` (a ROS *parameter* — a named value you can set at
  launch). Package default `0.25` m; the mission raises it to `0.40` m. The right
  value depends on the robot's shape and whether the arm pokes into the lidar's
  plane. **It must match `raytrace_min_range` in the navigation costmaps** (a
  navigation setting), otherwise obstacles in that close-in blind ring get erased
  as the robot approaches them.

### `slam_toolbox` (its sync node) — the SLAM engine *(installed, NOT ours)*
- **What kind of thing it is:** a node from an off-the-shelf package
  (`ros-humble-slam-toolbox`, installed with `apt`). We **configured** it; we did
  **not** write it.
- **Subscribes to** `/scan_filtered` and reads the `odom → base_link` transform as
  a rough starting guess of how the robot moved between scans.
- **Publishes** `/map` (the occupancy grid, message type
  `nav_msgs/OccupancyGrid`) and the `map → odom` transform ~50×/sec — the
  continuously-corrected answer to "where is the robot, really."
- **How it works in one sentence:** it lines up each new lidar scan against the
  walls it has already mapped ("scan matching") and nudges its position estimate
  so the scan fits — which corrects the drift that wheel odometry alone builds up.
- **Configured by** `slam_params.yaml` (next section).
- **Why `slam_toolbox` and not something else:** it's the standard, well-supported
  SLAM package for our ROS 2 version, and this exact config was already proven to
  map well on *this* robot by the workshop's reference launch. We picked the
  **sync** flavour of its node on purpose — see §6.

---

## 5. `slam_params.yaml` — the SLAM settings and the reasoning *(WE wrote this)*

This file is just a list of values handed to `slam_toolbox` at start-up. The ones
that matter:

| Setting | Value | Plain-English reason |
|---|---|---|
| `mode` | `mapping` | Build a brand-new map (we have no saved map to load). |
| `scan_topic` | `/scan_filtered` | Use the cleaned scan from our filter, **not** the raw `/scan`. |
| `use_scan_matching` | `true` | **The crucial one.** The mecanum wheels' odometry drifts a lot, worst when spinning in place. Scan-matching pulls the estimate back onto the real walls. This is *why every in-place spin in the mission is deliberately slow* (≤0.3–0.4 rad/s): spin too fast and the matcher can't keep up, so rotated, smeared copies of the room get stamped into the map. |
| `resolution` | `0.02` m per cell | Map detail. The value proven to work on this robot's onboard computer. |
| `map_update_interval` | `1.0` s | Redraw the occupancy grid once a second. |
| `transform_publish_period` | `0.02` s (50 Hz) | Emit the `map→odom` correction often, so the position estimate stays smooth. |
| `transform_timeout` | `0.5` s | Tolerate brief delays in scans/transforms when the computer is busy. |
| `minimum_travel_distance` / `_heading` | `0.05` m / `0.1` rad | How far the robot must move before SLAM bothers adding a new scan to its records. |
| `throttle_scans` | `1` | **Process every single scan** — never skip any. Skipped scans during a spin lose the matcher's lock. |

**If the onboard computer can't keep up** (only if `slam_toolbox` *continuously*
logs "queue is full" *and* the map visibly lags the robot): first raise
`resolution` to `0.03`, then `map_update_interval` to `2.0`. **Never** switch to
the async node or set `throttle_scans: 2` — that exact "optimisation" is what
smeared the map last time.

---

## 6. Why the **sync** SLAM node (a lesson we paid for)

`slam_toolbox` ships two interchangeable nodes: an **async** one and a **sync**
one. We use **sync**.

The async node, when the computer is busy, quietly **throws away** scans to keep
up. The busiest moments are exactly the in-place spins — which is exactly when a
dropped scan makes the matcher lose its lock and stamp a rotated, mis-aligned copy
of the room into the map. The **sync** node processes every scan in order, so it
never does that. We can afford the sync node's extra work because we lighten the
computer's load elsewhere (the marker detector runs on the laptop, and the
navigation stack is bundled into one process — both explained in README_NAV.md).

---

## 7. Mapping ownership — exactly what WE wrote vs. what we INSTALLED

The professor asked which files/packages/nodes are *ours*. For **mapping**:

| Thing | Type | Mapping or Nav? | Who made it |
|---|---|---|---|
| `mirte_driving_3/scan_filter.py` | node (a program) | **Mapping** | **WE wrote it** |
| `params/slam_params.yaml` | config (settings file) | **Mapping** | **WE wrote it** |
| `slam_toolbox` (sync node) | node, from an installed package | **Mapping** | Installed via `apt` — we only configured it |
| `odom → base_link` transform | a TF transform | shared dependency | The **robot's own base controller** — not us |
| `base_link → laser` transform | a TF transform | shared dependency | The **robot's own robot_state_publisher** — not us |
| `/scan`, `/map`, the camera | hardware topics/driver | shared dependency | The **robot's own bring-up** — not us |

So when presenting: *"For mapping we wrote one node, `scan_filter`, and the SLAM
configuration `slam_params.yaml`. The SLAM algorithm itself is the standard
`slam_toolbox` package, which we configured. The transforms it needs come from the
robot's own driver layer."*

> **Where the inflation radius lives — it is NOT mapping.** You may remember
> tweaking "inflation radius" and similar values. Those belong to **navigation**,
> not mapping. Mapping (SLAM) produces the *raw* map: walls free/occupied, full
> stop. **Inflation** is a navigation idea — it grows every obstacle outward by a
> safety margin so the planner keeps the robot's body clear. It lives in
> `params/exploration_nav2_params.yaml` and is changed per-leg by
> `shuttle_manager.py`. All of that is covered in **README_NAV.md §"Costmaps &
> inflation."** Short version: *map = mapping; costmap/inflation = navigation.*

---

## 8. How to check mapping is working (on its own)

```bash
# On the robot, with the stack running:
ros2 run tf2_ros tf2_echo map base_link   # must print real, changing numbers
ros2 topic echo /scan_filtered --once --field range_min   # filter is alive
ros2 topic hz /map                        # map is updating (~1 Hz)
```

In RViz (set "Fixed Frame" to `map`): add a **Map** display on `/map` — you should
see the room fill in as the robot drives; add a **LaserScan** display on
`/scan_filtered` — the dots should land on the walls, not on the robot. In the
mission, the `WAIT_SLAM` step blocks until `map → base_link` exists, then logs
`SLAM/TF ready`.

**Common failure → cause:**
- "Waiting for SLAM (map→base_link)…" forever → either no `/scan` (lidar or
  `scan_filter` down) **or** the robot isn't publishing `odom → base_link` (its
  base driver isn't up). Check `ros2 run tf2_ros tf2_echo odom base_link`.
- Map smears / rotated copies of the room → spinning too fast for the scan
  matcher, or the async node crept back in.
- Robot frozen, "collision ahead" everywhere → self-hits inside the body; raise
  `scan_min_range`.
- `TF_OLD_DATA` spam → two programs publishing the same transform. With our
  duplicates deleted this should not happen; if it does, something *outside* our
  package is double-publishing `odom → base_link` — check the robot's bring-up.
