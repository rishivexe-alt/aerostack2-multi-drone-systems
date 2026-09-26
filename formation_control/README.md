# Leader-Follower Formation Control

Three-drone formation flight on AeroStack2 / ROS 2 Humble / Gazebo. One leader (`drone0`) flies a predefined route; two followers (`drone1`, `drone2`) track it at fixed relative offsets, maintaining a consistent triangular ("delta") formation.

## Objective

Demonstrate coordinated multi-agent flight where follower targets are derived deterministically from a leader's planned position, and all three drones are commanded concurrently so the formation geometry holds during transit.

## Configuration

| Parameter | Value |
|---|---|
| Takeoff height | 1.0 m |
| Takeoff speed | 0.7 m/s |
| Mission speed | 0.5 m/s |
| Landing speed | 0.4 m/s |
| Formation stabilization time | 2.0 s |
| Follower 1 (`drone1`) offset | (+1.5, −1.5, 0.0) m |
| Follower 2 (`drone2`) offset | (−1.5, −1.5, 0.0) m |

**Mission route** — leader flies a closed square at 1.0 m altitude:

```
(0,0) → (2,0) → (2,2) → (0,2) → (0,0)
```

Follower targets are recomputed once per waypoint from the leader's *planned* (not live-measured) position — a fixed-offset, waypoint-synchronized formation strategy.

## How to Run

```bash
# 1. Source the environment
source /opt/ros/humble/setup.bash
source ~/aerostack2_ws/install/setup.bash

# 2. Launch the 3-drone Gazebo simulation
../scripts/launch_as2.bash -m

# 3. (Optional) live ground-station monitoring
../scripts/launch_ground_station.bash -m -v

# 4. Run the mission
python3 leader_follower_mission.py
```

The script pauses at three checkpoints (press ENTER to proceed):
1. Arm + takeoff
2. Start the formation mission (5 waypoints)
3. Land all drones

```bash
# 5. Stop the simulation
../scripts/stop.bash
```

## Verifying the Simulation Before Running

```bash
for d in drone0 drone1 drone2; do
    echo "========== $d =========="
    ros2 topic echo /$d/platform/info --once
done
```

Expect `connected: true`, `armed: false`, `offboard: false` prior to running the mission.

## Expected Output

```
TAKEOFF RESULTS
drone0: SUCCESS
drone1: SUCCESS
drone2: SUCCESS

ALL DRONES REACHED TAKEOFF ALTITUDE.
...
FORMATION MISSION COMPLETED SUCCESSFULLY
...
LANDING RESULTS
drone0: SUCCESS
drone1: SUCCESS
drone2: SUCCESS
```

All 5 waypoints should return `SUCCESS` for all 3 drones. Full results table in [`../docs/formation-control.md`](../docs/formation-control.md).

## Implementation Notes

- **`LeaderFollowerSystem`** owns all three `Drone` instances, assigns leader/follower roles, and drives the mission through `prepare_drones() → takeoff() → execute_mission() → land() → shutdown()`.
- **`move_formation()`** computes follower targets from the leader's target plus fixed offsets, then dispatches all three GoTo commands concurrently using one `threading.Thread` per drone, joined before advancing — this keeps the formation geometrically intact during transit rather than introducing a timing offset between drones.
- **GoTo call:** `drone.go_to.go_to(x, y, z, MISSION_SPEED, "earth")` — note the submodule method (`go_to.go_to()`, not `go_to()` directly).

```python
def execute_go_to(self, name, drone, position, results):
    x, y, z = position
    result = drone.go_to.go_to(x, y, z, MISSION_SPEED, "earth")
    results[name] = result
```

See [`leader_follower_mission.py`](leader_follower_mission.py) for the complete implementation.
