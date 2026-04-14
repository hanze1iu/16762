# 16-762 Final Project: Semantic Fetch

**Autonomous object fetch-and-place on Hello Robot Stretch 3**

---

## Goal

Build a full pipeline that takes a plain-text object name from the user and autonomously:

1. Looks up which semantic zone the object is likely to be in (from a pre-annotated SLAM map)
2. Navigates to that zone
3. Searches the zone with a two-phase scan strategy
4. Detects and grasps the object (YOLO-E + IK)
5. Transports it to a pre-defined drop-off location
6. Places it and reports success

```
User:  "fetch water bottle"
Robot: [navigates to kitchen] → [scans] → [detects bottle] → [grasps] → [navigates to dropoff] → [places]
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        CLI Entry Point                       │
│                     fetch.py  <object>                       │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                   Semantic Map Lookup                        │
│           semantic_map.yaml → zone + drop-off coords        │
└────────────────────────────┬────────────────────────────────┘
                             │  (zone nav goal)
                             ▼
┌─────────────────────────────────────────────────────────────┐
│              Navigation to Zone  (Nav2)                      │
│              reuse: stretch_navigation pattern               │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                    Search Behavior                           │
│  Phase 1: in-place head pan sweep  (head camera + YOLO-E)   │
│  Phase 2: orbit waypoints around zone + rescan at each pt   │
│  → Object found? → Grasp Pipeline                           │
│  → Still not found? → EXIT, report failure                  │
└────────────────────────────┬────────────────────────────────┘
                             │  (3D goal pose)
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                    Grasp Pipeline                            │
│  head camera → YOLO-E detect → pointcloud centroid → IK     │
│  reuse: object_detector_pcd.py + ik_ros_utils.py            │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│              Transport to Drop-off  (Nav2)                   │
│              navigate with object held (arm retracted)       │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│                      Placement                               │
│   navigate to drop-off → lower arm → open gripper → stow    │
└─────────────────────────────────────────────────────────────┘
```

---

## File Structure

```
project/
├── README.md                    ← this file
├── final_project/
│   ├── fetch.py                 ← CLI entry point / main orchestrator
│   ├── search_behavior.py       ← 360° base spin search (head camera + YOLO-E)
│   ├── grasp_pipeline.py        ← IK iterative approach + grasp (wraps lab 3)
│   ├── navigation_utils.py      ← Nav2 helpers (wraps lab 4 pattern)
│   ├── map_annotator.py         ← RViz click → semantic_map.yaml recorder (M1 tool)
│   ├── semantic_map.yaml        ← zone defs, object→zone priors, drop-off coords
│   └── map/
│       ├── <mapname>.pgm        ← SLAM occupancy grid
│       └── <mapname>.yaml       ← map metadata (resolution, origin)
```

---

## Semantic Map Design

### `semantic_map.yaml` — the core configuration file

```yaml
# Drop-off locations (predefined, named)
dropoffs:
  default:
    x: 0.5
    y: 0.2
    yaw: 0.0
  table:
    x: 1.2
    y: -0.5
    yaw: 1.57

# Named zones: center nav point + orbit waypoints for Phase 2 search
zones:
  kitchen:
    nav_point: {x: 2.1, y: -1.3, yaw: 0.0}
    orbit_waypoints:
      - {x: 2.3, y: -1.0, yaw: -1.57}
      - {x: 1.9, y: -1.0, yaw:  1.57}
      - {x: 2.1, y: -1.6, yaw:  0.0}

  study:
    nav_point: {x: 0.8, y:  1.5, yaw: 1.57}
    orbit_waypoints:
      - {x: 0.5, y: 1.7, yaw: 0.0}
      - {x: 1.1, y: 1.7, yaw: 3.14}

# Object → zone prior mapping
#   list multiple zones in priority order if the object could appear in more than one
object_zones:
  water bottle: [kitchen]
  coffee mug:   [kitchen, study]
  book:         [study]
  pen:          [study]
  apple:        [kitchen]
```

### How coordinates are determined

1. Build the SLAM map (see Milestone 1 below)
2. Use RViz to read off `(x, y)` positions by clicking on the map
3. Fill in `semantic_map.yaml` manually — zone centers, orbit waypoints, drop-off points

---

## Search Behavior (two-phase)

### Phase 1 — In-place head sweep
- Robot arrives at `zones.<zone>.nav_point`
- Sweep `joint_head_pan` across its full range while running YOLO-E on head camera frames
- If target detected at any pan angle → record 3D pose → proceed to grasp

### Phase 2 — Orbit + rescan
- Navigate sequentially through `zones.<zone>.orbit_waypoints`
- At each waypoint, repeat a shorter head sweep
- First detection → proceed to grasp
- All waypoints exhausted with no detection → EXIT, print `"[FAIL] <object> not found in <zone>"`

---

## Milestones

### M1 — SLAM Mapping + Semantic Annotation

**Step 1: Build the SLAM map**

Only 2 terminals needed — `offline_mapping.launch.py` already includes SLAM + RViz:

```bash
# Terminal 1 — robot driver
ros2 launch stretch_core stretch_driver.launch.py

# Terminal 2 — offline SLAM mapping (includes RViz, teleop keyboard by default)
ros2 launch stretch_nav2 offline_mapping.launch.py
```

Drive the robot through every area you want to include (RViz shows the map building in real time).
Once happy with coverage, save in a **new terminal**:

```bash
# Terminal 3 — save the map to the standard Stretch maps directory
mkdir ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/ai_maker_space
# → writes:  ai_maker_space.pgm
#            ai_maker_space.yaml

# Verify the map looks correct
eog ${HELLO_FLEET_PATH}/maps/ai_maker_space.pgm
```

**Step 2: Record semantic zone coordinates**

> This is a **continuation of the same session** — no need to go back to the robot a second time.
> T1 keeps running. T2 switches from mapping to navigation. T3 switches to the annotator.

```bash
# Terminal 1 — robot driver (keep running, nothing to change)

# Terminal 2 — Ctrl-C offline_mapping, then reuse the same pane for Nav2
ros2 launch stretch_nav2 navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/ai_maker_space.yaml

# Terminal 3 — reuse the same pane for the annotator
cd ~/path/to/final_project
python3 map_annotator.py
```

In RViz: click **"Publish Point"** (the crosshair icon) then click on the map.
The annotator prints the coordinates and prompts for a label:

```
[CLICK] x=2.103  y=-1.287  robot_yaw=0.0°
  Enter label (or ENTER to skip): zone kitchen
  [recorded] zone "kitchen" nav_point → {'x': 2.103, 'y': -1.287, 'yaw': 0.0}
  [saved] semantic_map.yaml
```

Label format:
- `zone <name>` — nav point of the zone (robot navigates here, then spins 360° to search)
- `dropoff <name>` — where the robot places the fetched object

Typical annotation session:
1. Record 1 × `zone <name>` per area (kitchen, study, etc.)
2. Record drop-off points

The file auto-saves after every click. Ctrl-C when done.

**Step 3: Add object→zone mappings**

Edit `final_project/semantic_map.yaml` and fill in `object_zones`:

```yaml
object_zones:
  water bottle: [kitchen]
  coffee mug:   [kitchen, study]
  book:         [study]
```

**Checklist**
- [ ] SLAM map built and saved to `${HELLO_FLEET_PATH}/maps/`
- [ ] `map_annotator.py` run; zone nav_points recorded
- [ ] Drop-off coordinates recorded
- [ ] `object_zones` filled in `semantic_map.yaml`

> **Status: TODO — running on robot 2025-04-12**

---

### M2 — CLI + Navigation to Zone

> **Status: CODE DONE — pending robot test**

**Files written:**
- `final_project/fetch.py` — CLI entry point, full pipeline orchestrator
- `final_project/navigation_utils.py` — Nav2 `BasicNavigator` wrapper (`go_to`, `follow_waypoints`)

**What works now:**
- `python3 fetch.py "water bottle"` loads the map, resolves object → zone(s), navigates to each zone nav_point in priority order
- If nav to a zone fails it skips to the next zone automatically

**To test once M1 map is ready:**
```bash
cd final_project
python3 fetch.py "water bottle"          # default drop-off
python3 fetch.py "coffee mug" --dropoff table
```
- [ ] Confirm robot reaches zone nav_point correctly
- [ ] Confirm correct zone is selected for each object name
- [ ] Confirm error message when object name is not in `semantic_map.yaml`

---

### M3 — Search Behavior

> **Status: CODE DONE — pending robot test**

**Files written:**
- `final_project/search_behavior.py` — `SearchBehavior` class

**What it does:**
- Attaches head-camera subscribers (`/camera/color`, `/camera/aligned_depth_to_color`, `/camera/color/camera_info`) to the FetchNode — not a separate node
- Fixes head forward and slightly downward, then rotates the base in 8 steps × 45° = 360°
- Runs YOLO-E at each stop; on first detection returns 3D goal pose
- 3D pose: rasterises the YOLO mask polygon → projects all interior pixels with valid depth → pointcloud centroid (lab 3 Part 2 approach)
- Coordinate fix for rotated head camera: `x_orig = y_rot`, `y_orig = h_orig - 1 - x_rot`
- TF-transforms result from camera frame → `base_link` before returning
- Returns `PoseStamped` in `base_link`, or `None` if nothing found after full spin

**To test (after M1 map is ready):**
```bash
cd final_project
python3 fetch.py "water bottle"   # should navigate → 360° spin → print detection result
```
- [ ] Confirm 360° spin runs without errors
- [ ] Confirm YOLO-E detects the object (check confidence / frame issues)
- [ ] Confirm 3D centroid is reasonable (print xyz before TF transform)

---

### M4 — Grasp Pipeline

> **Status: CODE DONE — pending robot test**

**Files written:**
- `final_project/grasp_pipeline.py` — `GraspPipeline` class

**What it does:**
- Subscribes to `/stretch/joint_states` and TF on the existing FetchNode — not a separate node
- Moves to ready pose (lift=0.8, arm retracted, gripper open, head forward)
- Iteratively approaches the goal in 5 cm steps (max 40 steps):
  - Reads current gripper pose from TF (`link_grasp_center` → `base_link`)
  - Computes waypoint toward goal with 5 cm safety stand-off along x
  - IK solve via `ik_ros_utils.get_grasp_goal()` → `move_to_configuration()`
  - When gripper is within 5 cm of target → close gripper (`gripper_aperture: -0.2`)
- Returns `True` on success, `False` on IK failure or timeout
- Plugged into `fetch.py` — `GraspPipeline(self)` created once before the zone loop

**To test (after M1 + M3 working):**
```bash
cd final_project
python3 fetch.py "water bottle"   # should navigate → search → approach → grasp
```
- [ ] Confirm ready pose is reached correctly
- [ ] Confirm IK finds solutions (check print output)
- [ ] Confirm gripper closes on the object
- [ ] Tune `SAFETY_X`, `DELTA`, `SETTLE_SEC` if needed

---

### M5 — Transport + Placement

> **Status: TODO**

- [ ] After grasp, retract arm to carry pose before navigating
- [ ] Navigate to drop-off via `nav.go_to()`  *(nav code already in fetch.py)*
- [ ] Lower arm to table height, open gripper, stow arm
- [ ] Plug into `fetch.py` `place_object()` stub
- [ ] Test: object placed at drop-off

---

### M6 — Integration + End-to-End Testing

> **Status: TODO**

- [ ] Replace all stubs in `fetch.py` with real M3/M4/M5 calls
- [ ] Test with 3+ different objects in 2+ zones
- [ ] Record video for presentation

---

## Reused Code from Labs

| Lab | File | Reused for |
|---|---|---|
| Lab 2 | `ik_ros_utils.py` | IK chain setup, `get_grasp_goal()`, `move_to_configuration()` |
| Lab 3 | `object_detector_pcd.py` | YOLO-E head camera detect + pointcloud centroid |
| Lab 3 | `grasp_objects.py` | Grasp execution, approach, close gripper |
| Lab 4 | `stretch_navigation.py` | Nav2 `BasicNavigator` pattern |

---

## Key Design Decisions

| Decision | Choice | Reason |
|---|---|---|
| Object→zone lookup | Static YAML prior | No VLM/training needed, fast, sufficient for demo |
| Detection model | YOLO-E (existing) | Already deployed on Stretch, zero-shot text prompts |
| Search strategy | 360° base spin at zone nav_point | Simple, no extra waypoints needed; covers full surroundings |
| Grasp method | Head camera + pointcloud centroid | Consistent with lab 3 Part 2, most robust depth |
| Transport pose | Arm retracted (stow-like) | Keeps CoM low and stable during navigation |
| Placement | Navigate to coord → lower → open gripper | Simple, no force sensing needed for demo |

---

## ROS2 Launch Checklist (before running)

```bash
# Terminal 1 — robot driver
ros2 launch stretch_core stretch_driver.launch.py

# Terminal 2 — head camera
ros2 launch stretch_core d435i_low_resolution.launch.py

# Terminal 3 — Nav2 + map
ros2 launch stretch_nav2 navigation.launch.py map:=<path>/map.yaml

# Terminal 4 — run fetch
python3 final_project/fetch.py "water bottle"
```

---

## Open Questions / Future Work

- **Multiple objects in one run**: current design is one object per invocation; could extend to a list
- **Dynamic re-mapping**: if layout changes, YAML needs manual update; could add a re-scan mode
- **Failure recovery**: if grasp fails (IK no solution, object slips), currently no retry; could add one retry with a slightly different approach angle
- **Drop-off placement accuracy**: currently nav to a coordinate and drop; could add a fiducial marker at drop-off for more precise placement
