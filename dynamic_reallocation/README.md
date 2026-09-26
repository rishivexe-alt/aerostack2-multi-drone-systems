# Dynamic Multi-Drone Area Survey & Task Reallocation

Three-drone autonomous area-survey mission on AeroStack2 / ROS 2 Humble / Gazebo, with runtime detection of a delayed drone, dynamic task transfer to a helper (preempting it if necessary), and resumption of the helper's own original mission.

## Objective

Demonstrate mission-level resilience: the survey field is split into three regions, each independently assigned. When one drone is deliberately delayed, a central Mission Manager identifies its unfinished waypoints, selects a helper, transfers the work, and later resumes the helper's own interrupted survey — with global, duplicate-free completion tracking.

## Configuration

| Parameter | Value | Purpose |
|---|---|---|
| Takeoff height | 1.0 m | Common survey altitude |
| Takeoff speed | 0.7 m/s | Initial ascent |
| Mission speed | 1.0 m/s | GoTo speed during survey |
| Landing speed | 0.4 m/s | Descent |
| Stabilization time | 3.0 s | Post-takeoff settle |
| Simulated delay | 8.0 s | Injected fault condition |
| Delay drone / waypoint | `drone0`, before waypoint 3 | Leaves A3 and A4 unfinished |
| Helper wait timeout | 5.0 s | Time to wait for a naturally idle helper |
| Preemption stop timeout | 8.0 s | Max time for a preempted thread to exit |
| GoTo timeout | 60.0 s | Safety cap per waypoint |

**Regions** (4 waypoints each, 1.0 m altitude):

| Drone | Area | Waypoints (x, y, z) m |
|---|---|---|
| `drone0` | A | (-2.5,-2.0,1.0) → (-2.5,2.0,1.0) → (-1.0,2.0,1.0) → (-1.0,-2.0,1.0) |
| `drone1` | B | (-0.5,-2.0,1.0) → (-0.5,2.0,1.0) → (0.5,2.0,1.0) → (0.5,-2.0,1.0) |
| `drone2` | C | (1.0,-2.0,1.0) → (1.0,2.0,1.0) → (2.5,2.0,1.0) → (2.5,-2.0,1.0) |

## How to Run

```bash
# 1. Source the environment
source /opt/ros/humble/setup.bash
source ~/aerostack2_ws/install/setup.bash

# 2. Launch the 3-drone Gazebo simulation
../scripts/launch_as2.bash -m

# 3. (Optional) live ground-station monitoring
../scripts/launch_ground_station.bash -m -v

# 4. Syntax check (optional)
python3 -m py_compile area_survey.py

# 5. Run the mission
python3 area_survey.py
```

```bash
# 6. Stop the simulation
../scripts/stop.bash
```

## Verifying the Simulation Before Running

```bash
for d in drone0 drone1 drone2; do
    echo "========== $d =========="
    ros2 topic echo /$d/platform/info --once
done

ros2 node list | grep -E '/drone0|/drone1|/drone2'
ros2 action list | grep -E '/drone0|/drone1|/drone2'
```

Confirm `GoToBehavior`, `TakeoffBehavior`, `LandBehavior`, `controller_manager`, `platform`, and `state_estimator` are present for all three namespaces.

## Mission Lifecycle

1. Initialize all three `DroneInterface` objects.
2. Arm and enable Offboard on all drones; concurrent takeoff to 1.0 m.
3. Start parallel surveys: `drone0→A`, `drone1→B`, `drone2→C`.
4. `drone0` completes waypoints 1–2, then triggers an 8-second simulated delay before waypoint 3.
5. Mission Manager waits up to 5 s for a naturally idle helper.
6. If none appears, it preempts the busy drone that has made the least progress on its own survey (in the validated run: `drone2`).
7. `drone0`'s remaining waypoints (A3, A4) are transferred to the helper.
8. Helper completes the transferred waypoints, then resumes its own original survey from where it was interrupted.
9. Mission Manager verifies full coverage (Area A/B/C = 4/4 each).
10. All drones land concurrently; interfaces shut down cleanly.

## Expected Output

```
DYNAMIC TASK REALLOCATION
Drone0 = DELAYED
Area A completed: 2/4
Drone0 remaining waypoints:
  A3: (-1.00, 2.00, 1.00)
  A4: (-1.00, -2.00, 1.00)

PREEMPTING BUSY HELPER
Selected helper: drone2
...
REALLOCATION COMPLETE
drone2 completed Drone0's remaining Area A work.
drone2 also completed its original Area C work.

FINAL MISSION VERIFICATION
Area A: 4/4
Area B: 4/4
Area C: 4/4
Dynamic reallocation: COMPLETED
Helper drone: drone2

MISSION COMPLETED SUCCESSFULLY
```

Full annotated screenshots of this exact run are in [`../docs/dynamic-reallocation.md`](../docs/dynamic-reallocation.md) and [`../docs/images/reallocation/`](../docs/images/reallocation/).

## Implementation Notes

- **`AreaSurveyManager`** is the central coordinator: delay detection, helper selection, preemption, task transfer, and final verification all live here.
- **Asynchronous GoTo** is essential — `SurveyDrone.execute_go_to()` starts the GoTo with `wait=False` and polls `go_to.is_running()` in a 0.1 s loop, so the manager can interrupt it mid-flight for preemption.

```python
accepted = self.go_to(
    x, y, z,
    MISSION_SPEED,
    0, None,
    "earth",
    False          # wait=False — required for responsive preemption
)
while self.go_to.is_running():
    time.sleep(0.1)
```

- **Global completion accounting:** `AreaSurveyManager.completed_work` is a set of `(area_name, index)` tuples, independent of which drone executed each waypoint — this is what allows Area A to be finished by two different physical drones without double-counting.
- **Helper selection:** `select_busy_helper()` prefers the busy drone that has completed the *fewest* waypoints of its own survey, minimizing disruption to work already done.
- **State machine:** each `SurveyDrone` moves through `SURVEYING → DELAYED → PREEMPTED → REASSIGNED → TRANSFERRED / AVAILABLE`, with all mutable state behind a `threading.Lock`.

See [`area_survey.py`](area_survey.py) for the complete implementation.
