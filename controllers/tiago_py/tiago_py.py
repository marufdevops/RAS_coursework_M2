# Author: Ahmed Maruf, SID: 250046920
# TiaGo Robot Navigation System with PDDL-based Path Planning
#
# This implementation uses a comprehensive PDDL-based navigation system supporting
# all 4 goals with the following features:
#
# - Expanded 11-node network with 16 weighted bidirectional connections
# - PDDL domain and problem generation for optimal path planning
# - Pyperplan integration with A* algorithm for shortest paths
# - Robust navigation controller with rotation and movement control
# - Path cost tracking and goal detection using goalchecker.py
# - 0.8m distance threshold for goal completion
# - Multi-goal support: balls, green, ducks, red
#
# Navigation Architecture:
# 1. Node Network: 11 predefined nodes with 16 weighted bidirectional connections
# 2. PDDL Planning: Generate domain/problem files and use pyperplan for optimal paths
# 3. Navigation Control: Follow planned path through nodes with cost tracking
# 4. Goal Detection: Use goalchecker.py to confirm target reached
#
# Usage: Type "balls", "green", "ducks", or "red" in the 3D view and press enter
# Expected Output: "Goal reached! Total distance covered: X.XX meters"
# References:
#    - Introduction to AI Robotics - Murphy, R.R
#        - Discusses about sate machines architecture and the basics of sensors
#    - Path and Motion Planning (class lectures, (2024-25 CS4790J) Robotics and Autonomous Systems, 
#      Aston University)
#    - 
#    - I have taken further helps from multiple LLM's
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

print("TiaGo PDDL Navigation System with Goal Queue Initialized")
print("=" * 60)
print("Supported targets: 'balls', 'green', 'ducks', 'red'")
print("Usage:")
print("  Single goal: Type 'balls' and press enter")
print("  Multiple goals: Type 'balls green red' and press enter")
print("  Add to queue: Type goals anytime (even during navigation)")
print("Expected output: 'Goal reached! Total distance covered: X.XX meters'")
print("=" * 60)

# ============================================================================
# MAIN CONTROL LOOP - Multi-Goal PDDL Navigation System
# ============================================================================
# This loop implements the primary control logic for the advanced multi-goal
# navigation system, integrating user input processing, PDDL path planning,
# and real-time navigation control with comprehensive error handling.

while robot.step(timestep) != -1:
    # ========================================================================
    # COMMAND PROCESSING SUBSYSTEM
    # ========================================================================
    # Process keyboard input for multi-goal navigation commands
    # Design Rationale: Asynchronous command processing prevents blocking
    # during navigation execution while maintaining responsive user interaction

    command = keyboard.get_command()
    if command is not None:
        print(f'Got command: {command}')

        # Parse command for multiple goals
        command_lower = command.strip().lower()
        goals = command_lower.split()

        # Filter valid goals
        valid_goals = [goal for goal in goals if goal in ['balls', 'green', 'ducks', 'red']]
        invalid_goals = [goal for goal in goals if goal not in ['balls', 'green', 'ducks', 'red']]

        if invalid_goals:
            print(f"❌ Invalid goals ignored: {invalid_goals}")

        if valid_goals:
            # Handle single vs multiple goals
            if len(valid_goals) == 1:
                single_goal = valid_goals[0]

                # Check if robot is idle or can start new navigation
                if nav_controller.state in ["IDLE", "GOAL_REACHED"]:
                    # Start single goal navigation
                    success = nav_controller.start_navigation_to_target(single_goal)
                    if not success:
                        print(f"❌ Failed to start navigation to {single_goal} target")
                else:
                    # Add to queue if navigation is in progress
                    nav_controller.add_goal_to_queue(single_goal)
                    print(f"🔄 Navigation in progress. Added '{single_goal}' to queue.")

            else:
                # Multiple goals - handle queue navigation
                if nav_controller.state in ["IDLE", "GOAL_REACHED"]:
                    # Start queue navigation
                    success = nav_controller.start_queue_navigation(valid_goals)
                    if not success:
                        print(f"❌ Failed to start queue navigation")
                else:
                    # Add all goals to existing queue
                    nav_controller.add_multiple_goals_to_queue(valid_goals)
                    print(f"🔄 Navigation in progress. Added goals to queue.")
        else:
            print(f"❌ No valid goals found. Supported: 'balls', 'green', 'ducks', 'red'")

    # ========================================================================
    # NAVIGATION CONTROL SUBSYSTEM
    # ========================================================================
    # Execute navigation controller update cycle
    # Performance Consideration: Single update call per timestep maintains
    # real-time performance while ensuring consistent control loop timing
    nav_controller.update()

    # ========================================================================
    # GOAL DETECTION AND VALIDATION SUBSYSTEM
    # ========================================================================
    # Real-time goal proximity detection using goalchecker.py integration
    # Design Rationale: Independent goal detection provides validation of
    # navigation success and enables debugging of positioning accuracy

    robot_pos = nav_controller.get_robot_position()
    nearby_goals = get_goals_in_range(*robot_pos)  # 0.8m detection threshold

    if nearby_goals:
        # Multi-goal proximity validation with navigation state awareness
        # Algorithmic Decision: Only report goals that match current navigation targets
        # to prevent spurious detection messages during complex maneuvers
        for goal in nearby_goals:
            if goal in ['balls', 'green', 'ducks', 'red'] and not nav_controller.goal_reached:
                # Debug feedback: Confirms goal detection system functionality
                print(f'Robot detected near {goal} target!')

    # ========================================================================
    # SYSTEM MONITORING AND DIAGNOSTICS
    # ========================================================================
    # Periodic status reporting for system monitoring and performance analysis
    # Timing Decision: 10-second intervals balance information density with
    # log readability while providing sufficient monitoring granularity

    if robot.getTime() % 10 < timestep / 1000.0:
        status = nav_controller.get_status()

        # Enhanced status reporting with queue information
        if status['state'] != "IDLE":
            # Basic navigation status
            status_msg = f"Status: {status['state']}"

            if status['current_goal']:
                status_msg += f", Goal: {status['current_goal']}"

            if status['current_target']:
                status_msg += f", Target: {status['current_target']}"

            status_msg += f", Distance: {status['distance_covered']:.2f}m"

            # Add queue information if there are queued goals
            if status['queue'] or status['total_goals'] > 1:
                status_msg += f", Progress: {status['progress']}"
                if status['queue']:
                    status_msg += f", Queue: {status['queue']}"

            print(status_msg)
    


