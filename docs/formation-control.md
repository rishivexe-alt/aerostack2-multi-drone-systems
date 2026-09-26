# Formation Control — Full Report

**3-Drone Leader-Follower Simulation** · ROS 2 Humble + Gazebo Based Multi-Drone Formation Control

## Executive Summary

The system uses one leader drone (`drone0`) and two follower drones (`drone1`, `drone2`). The leader is commanded along a predefined five-waypoint route; follower targets are computed at each waypoint by applying fixed `(x, y, z)` offsets to the leader's planned position, producing a triangular ("delta") formation. Arming and Offboard activation are performed sequentially, one drone at a time; takeoff, formation GoTo commands, and landing are then dispatched concurrently using one Python thread per drone so that all three drones begin moving toward their respective targets at the same time.

The final validated run completed the full mission lifecycle: ROS 2 interface initialization, arming, Offboard activation, concurrent takeoff to 1.0 m, five formation waypoints (all returning `SUCCESS` for all three drones), return to the initial waypoint, coordinated landing, and clean interface shutdown. The mission script printed **"FORMATION MISSION COMPLETED SUCCESSFULLY"** after the final waypoint, confirmed independently by a post-mission `/platform/info` check on all three drones.

## System Overview

The system runs entirely on AeroStack2 over ROS 2 Humble, with Gazebo providing physics simulation for three quadrotor models. Each simulated drone is exposed as an independent ROS 2 namespace (`/drone0`, `/drone1`, `/drone2`), wrapped by an AeroStack2 `DroneInterface` object in Python.

A `LeaderFollowerSystem` class owns all three `Drone` objects, designates `drone0` as the leader, and computes follower targets from fixed relative offsets. Arming and Offboard activation are performed sequentially per drone; takeoff, GoTo, and landing are dispatched concurrently, one Python thread per drone.

Each `DroneInterface` exposes: `arm()`, `offboard()`, `takeoff()`, `go_to.go_to()`, `land()`. The GoTo action is backed by the `as2_msgs/action/GoToWaypoint` action server, reporting feedback such as `actual_speed` and `actual_distance_to_goal` while in transit.

## Software and Simulation Environment

| Item | Configuration |
|---|---|
| Operating System | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Simulator | Gazebo |
| Framework | AeroStack2 |
| Main mission script | `leader_follower_mission.py` |
| World configuration | `config/world_swarm.yaml` |
| Drone model | `quadrotor_base` |

## Formation Control Strategy

Given a leader target position `L = (Lx, Ly, Lz)`, follower targets are computed by adding a fixed offset:

```
F1 = (Lx + 1.5, Ly - 1.5, Lz)   # Follower 1 (drone1)
F2 = (Lx - 1.5, Ly - 1.5, Lz)   # Follower 2 (drone2)
```

Because both followers are offset behind and to either side of the leader by the same magnitude in `y`, and split symmetrically in `x`, the three drones maintain a consistent triangular formation as the leader advances through each waypoint. This is explicitly a **fixed-offset, waypoint-synchronized** formation strategy — not a continuous controller reading the leader's live pose in real time.

## Mission Waypoint Design

| WP | Leader | Follower 1 | Follower 2 |
|---|---|---|---|
| 1 | (0.00, 0.00, 1.00) | (1.50, -1.50, 1.00) | (-1.50, -1.50, 1.00) |
| 2 | (2.00, 0.00, 1.00) | (3.50, -1.50, 1.00) | (0.50, -1.50, 1.00) |
| 3 | (2.00, 2.00, 1.00) | (3.50, 0.50, 1.00) | (0.50, 0.50, 1.00) |
| 4 | (0.00, 2.00, 1.00) | (1.50, 0.50, 1.00) | (-1.50, 0.50, 1.00) |
| 5 | (0.00, 0.00, 1.00) | (1.50, -1.50, 1.00) | (-1.50, -1.50, 1.00) |

Leader route: `(0,0) → (2,0) → (2,2) → (0,2) → (0,0)` — a closed square at 1.0 m altitude.

## Implementation

| Component | Responsibility |
|---|---|
| `Drone(DroneInterface)` | Thin subclass configuring each drone with verbose logging and simulation time. |
| `LeaderFollowerSystem` | Owns all three `Drone` objects; assigns roles; drives the mission lifecycle. |
| `prepare_drones()` | Arms and enables Offboard sequentially: `drone0`, then `drone1`, then `drone2`. |
| `takeoff()` / `takeoff_drone()` | Concurrent takeoff, one thread per drone. |
| `move_formation()` | Computes follower targets from the leader target + offsets, dispatches concurrent GoTo commands. |
| `execute_go_to()` | Issues one drone's GoTo command and records its result. |
| `execute_mission()` | Iterates the five waypoints, calling `move_formation()` for each. |
| `land()` / `land_drone()` | Concurrent landing. |
| `shutdown()` | Clean interface shutdown at mission end or on error. |

Takeoff, GoTo, and landing are all dispatched using `threading.Thread`, one per drone, joined before the system proceeds to the next stage — issuing them sequentially would introduce a timing offset between drones and break the formation geometry during transit.

## Validation Results

**Arming and Offboard:**
```
drone0: arm result = True    drone0: offboard result = True
drone1: arm result = True    drone1: offboard result = True
drone2: arm result = True    drone2: offboard result = True
All drones prepared.
```

**Takeoff:**
```
drone0: SUCCESS
drone1: SUCCESS
drone2: SUCCESS
ALL DRONES REACHED TAKEOFF ALTITUDE.
```

**Formation mission — all 5 waypoints, all 3 drones:**

| Waypoint | drone0 | drone1 | drone2 | Formation status |
|---|---|---|---|---|
| WP1 | SUCCESS | SUCCESS | SUCCESS | Completed |
| WP2 | SUCCESS | SUCCESS | SUCCESS | Completed |
| WP3 | SUCCESS | SUCCESS | SUCCESS | Completed |
| WP4 | SUCCESS | SUCCESS | SUCCESS | Completed |
| WP5 | SUCCESS | SUCCESS | SUCCESS | Completed |

**Landing:**
```
drone0: SUCCESS
drone1: SUCCESS
drone2: SUCCESS
All landing operations completed.
```

**Final `/platform/info` verification** (post-landing): all three drones `connected: true`, `armed: false`.

## Technical Analysis

- **Multi-agent initialization:** three independent `DroneInterface` objects under separate ROS 2 namespaces in a single Python process, all `connected = true` prior to any flight command.
- **Sequential prep, concurrent execution:** arming/Offboard sequential; takeoff/GoTo/landing concurrent via one thread per drone, joined before advancing — keeps all three drones synchronized within each motion stage.
- **Deterministic formation geometry:** the same triangular shape is reproduced at every waypoint from fixed offsets, without a separate formation-tracking control loop.
- **Action feedback as evidence:** `GoToWaypoint` feedback (`actual_speed`, `actual_distance_to_goal`) confirms genuine motion, not just an instantaneous success flag.

## Engineering Notes

During development, a GoTo API argument-ordering issue was debugged by tracing the call through to the correct submodule method: `go_to.go_to()`, not `go_to()` directly. No final position-accuracy figure (steady-state error in meters) was captured during this run — only qualitative in-transit feedback and the final `True`/`SUCCESS` results are claimed.
