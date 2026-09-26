# Shared Launch Scripts

Thin wrappers around AeroStack2's own launch tooling, shared by both `formation_control/` and `dynamic_reallocation/`.

| Script | Purpose |
|---|---|
| `launch_as2.bash -m` | Launches Gazebo with the 3-drone swarm world and brings up the AeroStack2 stack for each drone namespace. |
| `launch_ground_station.bash -m -v` | Optional live ground-station viewer (alphanumeric drone status) for monitoring a run. |
| `stop.bash` | Stops the simulation and associated AeroStack2/Gazebo processes. |

## Before using these

These scripts are **repo-level templates**. They assume:

- An AeroStack2 workspace at `~/aerostack2_ws` (override with the `AEROSTACK2_WS` environment variable)
- A swarm world config at `dynamic_reallocation/config/world_swarm.yaml` (override with `WORLD_CONFIG`)

Replace the `TODO` lines in `launch_as2.bash` and `launch_ground_station.bash` with your own AeroStack2 launch invocations (e.g. `ros2 launch as2_gazebo_assets launch_simulation.py ...`), matching how your local AeroStack2 install launches its Gazebo assets and ground-station tools.

```bash
export AEROSTACK2_WS=~/my_aerostack2_ws
./launch_as2.bash -m
```
