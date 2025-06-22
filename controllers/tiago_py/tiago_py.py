#
# TiaGo Robot Navigation System with PDDL-based Path Planning
#
# This implementation uses a simplified PDDL-based navigation system focusing
# on the "balls" target with the following features:
#
# - Graph-based node network with weighted connections
# - PDDL domain and problem generation for optimal path planning
# - Pyperplan integration with A* algorithm for shortest paths
# - Robust navigation controller with rotation and movement control
# - Path cost tracking and goal detection using goalchecker.py
# - 0.8m distance threshold for goal completion
#
# Navigation Architecture:
# 1. Node Network: 4 predefined nodes with weighted bidirectional connections
# 2. PDDL Planning: Generate domain/problem files and use pyperplan for optimal paths
# 3. Navigation Control: Follow planned path through nodes with cost tracking
# 4. Goal Detection: Use goalchecker.py to confirm balls target reached
#
# Usage: Type "balls" in the 3D view and press enter to start navigation
# Expected Output: "Goal reached! Total distance covered: X.XX meters"
#

from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range
from navigation_controller import NavigationController

# Initialize robot and navigation system
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize navigation controller
nav_controller = NavigationController(robot, timestep)
keyboard = KeyboardReader(timestep)

print("TiaGo PDDL Navigation System Initialized")
print("=" * 50)
print("Supported targets: 'balls' and 'green'")
print("Type 'balls' or 'green' in 3D view and press enter to start navigation")
print("Expected output: 'Goal reached! Total distance covered: X.XX meters'")
print("=" * 50)

# Main control loop
while robot.step(timestep) != -1:
    # Handle keyboard input for navigation commands
    command = keyboard.get_command()
    if command is not None:
        print(f'Got command: {command}')

        # Parse command - accept "balls" or "green"
        command_lower = command.strip().lower()
        if command_lower in ['balls', 'green']:
            if nav_controller.state == "IDLE":
                success = nav_controller.start_navigation_to_target(command_lower)
                if not success:
                    print(f"Failed to start navigation to {command_lower} target")
            else:
                print(f"Navigation already in progress (state: {nav_controller.state})")
        else:
            print(f"Invalid command: '{command}'. Supported targets: 'balls', 'green'")

    # Update navigation controller
    nav_controller.update()

    # Check if robot is near any goals (for debugging)
    robot_pos = nav_controller.get_robot_position()
    nearby_goals = get_goals_in_range(*robot_pos)
    if nearby_goals:
        for goal in nearby_goals:
            if goal in ['balls', 'green'] and not nav_controller.goal_reached:
                print(f'Robot detected near {goal} target!')

    # Print status every 10 seconds (for debugging)
    if robot.getTime() % 10 < timestep / 1000.0:
        status = nav_controller.get_status()
        if status['state'] != "IDLE":
            print(f"Status: {status['state']}, Target: {status['current_target']}, "
                  f"Distance covered: {status['distance_covered']:.2f}m")
    


