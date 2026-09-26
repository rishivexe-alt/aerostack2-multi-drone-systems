#!/usr/bin/env python3

import threading
import time

import rclpy

from aerostack2 import DroneInterface


TAKEOFF_HEIGHT = 1.0
TAKEOFF_SPEED = 0.7
LAND_SPEED = 0.4
MISSION_SPEED = 0.5

POSITION_TOLERANCE = 0.40
FORMATION_STABILIZATION_TIME = 2.0

FOLLOWER_1_OFFSET = (1.5, -1.5, 0.0)
FOLLOWER_2_OFFSET = (-1.5, -1.5, 0.0)

MISSION_WAYPOINTS = [
    (0.0, 0.0, 1.0),
    (2.0, 0.0, 1.0),
    (2.0, 2.0, 1.0),
    (0.0, 2.0, 1.0),
    (0.0, 0.0, 1.0),
]


class Drone(DroneInterface):
    def __init__(self, namespace):
        super().__init__(
            namespace,
            verbose=True,
            use_sim_time=True
        )


class LeaderFollowerSystem:
    def __init__(self):
        self.drones = {
            "drone0": Drone("drone0"),
            "drone1": Drone("drone1"),
            "drone2": Drone("drone2"),
        }

        self.leader = self.drones["drone0"]
        self.follower1 = self.drones["drone1"]
        self.follower2 = self.drones["drone2"]

        print("\n========================================")
        print("AEROSTACK2")
        print("3-DRONE LEADER-FOLLOWER SYSTEM")
        print("========================================")

        print("\nLeader   : drone0")
        print("Follower : drone1")
        print("Follower : drone2")

        print("\nFormation:")
        print("      drone0")
        print("     /      \\")
        print("  drone2    drone1")

        print("\n========================================")

    def prepare_drones(self):
        print("\n========================================")
        print("ARMING ALL DRONES")
        print("========================================")

        for name, drone in self.drones.items():
            print(f"{name}: arming...")
            result = drone.arm()
            print(f"{name}: arm result = {result}")

            print(f"{name}: enabling offboard...")
            result = drone.offboard()
            print(f"{name}: offboard result = {result}")

        print("\nAll drones prepared.")

    def takeoff_drone(self, name, drone, results):
        print(f"{name}: TAKEOFF -> {TAKEOFF_HEIGHT:.2f} m")
        result = drone.takeoff(
            TAKEOFF_HEIGHT,
            TAKEOFF_SPEED,
            False
        )
        results[name] = result
        print(f"{name}: takeoff command completed -> {result}")

    def takeoff(self):
        print("\n========================================")
        print("TAKEOFF")
        print("========================================")

        results = {}
        threads = []

        for name, drone in self.drones.items():
            thread = threading.Thread(
                target=self.takeoff_drone,
                args=(name, drone, results)
            )
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        print("\n========================================")
        print("TAKEOFF RESULTS")
        print("========================================")

        for name in self.drones:
            status = "SUCCESS" if results.get(name) else "FAILED"
            print(f"{name}: {status}")

        if all(results.get(name) for name in self.drones):
            print("\nALL DRONES REACHED TAKEOFF ALTITUDE.")
        else:
            print("\nWARNING: ONE OR MORE TAKEOFF COMMANDS FAILED.")

    def execute_go_to(self, name, drone, position, results):
        x, y, z = position

        print(f"{name}: GoTo -> ({x:.2f}, {y:.2f}, {z:.2f})")

        result = drone.go_to.go_to(
            x,
            y,
            z,
            MISSION_SPEED,
            "earth"
        )

        results[name] = result
        print(f"{name}: GoTo result = {result}")

    def move_formation(self, leader_position):
        lx, ly, lz = leader_position

        follower1_position = (
            lx + FOLLOWER_1_OFFSET[0],
            ly + FOLLOWER_1_OFFSET[1],
            lz + FOLLOWER_1_OFFSET[2]
        )

        follower2_position = (
            lx + FOLLOWER_2_OFFSET[0],
            ly + FOLLOWER_2_OFFSET[1],
            lz + FOLLOWER_2_OFFSET[2]
        )

        targets = {
            "drone0": leader_position,
            "drone1": follower1_position,
            "drone2": follower2_position,
        }

        print("\n--------------------------------------------------")
        print("FORMATION UPDATE")
        print("--------------------------------------------------")

        print(f"Leader     -> ({leader_position[0]:.2f}, {leader_position[1]:.2f}, {leader_position[2]:.2f})")
        print(f"Follower 1 -> ({follower1_position[0]:.2f}, {follower1_position[1]:.2f}, {follower1_position[2]:.2f})")
        print(f"Follower 2 -> ({follower2_position[0]:.2f}, {follower2_position[1]:.2f}, {follower2_position[2]:.2f})")

        print("\nStarting all three GoTo commands...")

        results = {}
        threads = []

        for name, drone in self.drones.items():
            thread = threading.Thread(
                target=self.execute_go_to,
                args=(name, drone, targets[name], results)
            )
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        print("\n--------------------------------------------------")
        print("WAYPOINT RESULTS")
        print("--------------------------------------------------")

        for name in self.drones:
            status = "SUCCESS" if results.get(name) else "FAILED"
            print(f"{name}: {status}")

        print(f"\nStabilizing formation for {FORMATION_STABILIZATION_TIME:.1f} seconds...")
        time.sleep(FORMATION_STABILIZATION_TIME)

        return all(results.get(name) for name in self.drones)

    def execute_mission(self):
        print("\n========================================")
        print("LEADER-FOLLOWER FORMATION MISSION")
        print("========================================")

        for index, waypoint in enumerate(MISSION_WAYPOINTS, start=1):
            print("\n========================================")
            print(f"WAYPOINT {index}/{len(MISSION_WAYPOINTS)}")
            print("========================================")

            print(f"Leader target = ({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f})")

            success = self.move_formation(waypoint)

            if success:
                print(f"\nWaypoint {index} completed successfully.")
            else:
                print(f"\nWaypoint {index} had one or more failures.")

        print("\n========================================")
        print("FORMATION MISSION COMPLETED SUCCESSFULLY")
        print("========================================")

    def land_drone(self, name, drone, results):
        print(f"{name}: LANDING...")
        result = drone.land(
            LAND_SPEED,
            False
        )
        results[name] = result
        print(f"{name}: landing command completed -> {result}")

    def land(self):
        print("\n========================================")
        print("LANDING ALL DRONES")
        print("========================================")

        results = {}
        threads = []

        for name, drone in self.drones.items():
            thread = threading.Thread(
                target=self.land_drone,
                args=(name, drone, results)
            )
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        print("\n========================================")
        print("LANDING RESULTS")
        print("========================================")

        for name in self.drones:
            status = "SUCCESS" if results.get(name) else "FAILED"
            print(f"{name}: {status}")

        print("\nAll landing operations completed.")

    def shutdown(self):
        print("\n========================================")
        print("SHUTTING DOWN DRONE INTERFACES")
        print("========================================")

        for drone in self.drones.values():
            try:
                drone.shutdown()
            except Exception:
                pass


def main():
    rclpy.init()

    swarm = LeaderFollowerSystem()

    try:
        input("\nPress ENTER to ARM and TAKEOFF...")

        swarm.prepare_drones()

        swarm.takeoff()

        input("\nPress ENTER to start LEADER-FOLLOWER FORMATION...")

        swarm.execute_mission()

        input("\nPress ENTER to LAND all drones...")

        swarm.land()

    except KeyboardInterrupt:
        print("\nMission interrupted by user.")

    except Exception as e:
        print(f"\nMission error: {e}")

    finally:
        swarm.shutdown()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()