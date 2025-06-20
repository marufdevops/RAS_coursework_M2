"""
Mission 2: Navigation with Reactive Obstacle Avoidance

TiaGo Robot Navigation System with PDDL-based Global Planning and Reactive Obstacle Avoidance

SYSTEM ARCHITECTURE:
This implementation uses a sophisticated subsumption architecture that coordinates:
1. PDDL-based global path planning for strategic navigation between warehouse locations
2. Reactive obstacle avoidance with target memory for robust collision avoidance
3. Task queue management for handling multiple sequential navigation commands
4. Low-level motor control with velocity smoothing and acceleration limiting

NAVIGATION APPROACH:
- Global Planning: Uses PDDL domain/problem files with pyperplan for optimal path generation
- Local Navigation: Waypoint-based navigation with proportional control
- Obstacle Avoidance: Subsumption-based reactive behavior with state memory
- Sensor Processing: LiDAR obstacle detection, GPS localization, compass orientation

TARGET LOCATIONS:
- Red Box: (3.685, 3.01) - Right room, upper area
- Green Box: (3.685, -3.94) - Right room, lower area
- Ducks Container: (-3.07, 3.64) - Left room, upper area
- Balls Container: (-2.795, -4.17) - Left room, lower area

KEYBOARD INTERFACE:
- r/R: Navigate to red box (R = high priority)
- g/G: Navigate to green box (G = high priority)
- d/D: Navigate to ducks container (D = high priority)
- b/B: Navigate to balls container (B = high priority)
- c/C: Cancel current task
- q/Q: Clear task queue
- s/S: Show status

DESIGN RATIONALE:
The system implements a hierarchical behavior architecture where:
1. Global planning provides strategic waypoints using PDDL deliberative planning
2. Local navigation handles waypoint-to-waypoint movement with obstacle awareness
3. Reactive avoidance takes precedence when obstacles are detected
4. Target memory ensures return to original goal after avoidance maneuvers

This approach balances deliberative planning efficiency with reactive robustness,
ensuring successful navigation in dynamic environments with multiple obstacles.

Author: Mission 2 Navigation System
Date: 2025-06-19
"""

import math
import time
from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range

# Import navigation system components
from behavior_coordinator import BehaviorCoordinator
from task_manager import TaskManager
from navigation_controller import NavigationController

# Initialize robot and basic parameters
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize sensors
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)

# Initialize keyboard input
keyboard = KeyboardReader(timestep)

# Initialize navigation system components
behavior_coordinator = BehaviorCoordinator()
task_manager = TaskManager(keyboard)
navigation_controller = NavigationController(robot)

# System state variables
current_target = None
navigation_active = False
last_status_time = time.time()
status_interval = 5.0  # Print status every 5 seconds

print("=== TiaGo Navigation System Initialized ===")
print("Available commands: r/R (red), g/G (green), d/D (ducks), b/B (balls)")
print("Control commands: c/C (cancel), q/Q (clear queue), s/S (status)")
print("System ready for navigation tasks...")

# Main control loop
while robot.step(timestep) != -1:
    try:
        # Get sensor data
        gps_values = gps.getValues()
        compass_values = compass.getValues()
        lidar_values = lidar.getRangeImage()

        # Process keyboard input and task management
        new_target = task_manager.process_keyboard_input()

        # Start new navigation task if available
        if new_target and not navigation_active:
            try:
                if behavior_coordinator.set_navigation_target(new_target):
                    current_target = new_target
                    navigation_active = True
                    print(f"Starting navigation to: {new_target}")
                else:
                    print(f"Failed to start navigation to: {new_target}")
                    task_manager.fail_current_task()
            except Exception as e:
                print(f"Error setting navigation target: {e}")
                task_manager.fail_current_task()

        # Update navigation behavior if active
        if navigation_active:
            # Get control commands from behavior coordinator
            linear_vel, angular_vel = behavior_coordinator.update(
                gps_values, compass_values, lidar_values
            )

            # Apply control commands through navigation controller
            navigation_controller.set_velocity(linear_vel, angular_vel)

            # Check if navigation is complete
            if behavior_coordinator.is_navigation_complete():
                print(f"Navigation to {current_target} completed successfully!")
                task_manager.complete_current_task()
                behavior_coordinator.reset_navigation()
                navigation_active = False
                current_target = None

                # Check for goals in range
                goals_in_range = get_goals_in_range(gps_values[0], gps_values[1])
                if goals_in_range:
                    print(f"Robot is now close to: {goals_in_range}")
        else:
            # No active navigation, stop the robot
            navigation_controller.stop()

        # Periodic status reporting
        current_time = time.time()
        if current_time - last_status_time > status_interval:
            last_status_time = current_time

            # Print system status
            position = gps_values[:2]
            goals_nearby = get_goals_in_range(position[0], position[1])

            print(f"\n--- System Status ---")
            print(f"Position: ({position[0]:.2f}, {position[1]:.2f})")
            print(f"Current Target: {current_target or 'None'}")
            print(f"Navigation Active: {navigation_active}")
            print(f"Goals in Range: {goals_nearby}")

            # Print task queue status
            queue_status = task_manager.get_queue_status()
            print(f"Queue Length: {queue_status['queue_length']}")
            print(f"Total Tasks: {queue_status['statistics']['total_tasks']}")
            print(f"Completed: {queue_status['statistics']['completed_tasks']}")
            print("--------------------\n")

    except Exception as e:
        print(f"Error in main loop: {e}")
        # Emergency stop on error
        navigation_controller.emergency_stop_enable()
        time.sleep(1)
        navigation_controller.emergency_stop_disable()