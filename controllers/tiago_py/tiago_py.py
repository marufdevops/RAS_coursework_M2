"""
This module implements a sophisticated multi-goal navigation system for the TiaGo robot,
integrating PDDL (Planning Domain Definition Language) path planning with real-time
obstacle avoidance and goal queue management.

**System Architecture:**
- PDDL-based optimal path planning using A* search algorithm
- Node network navigation with 11 strategically placed waypoints
- Multi-goal queue system for sequential target navigation
- Real-time obstacle detection and avoidance using LiDAR
- Precise goal detection using geometric polygon validation
- Precise positioning with 0.8m goal completion threshold

**Supported Goals:**
- 'balls': Soccer ball collection area (bottom-left region)
- 'green': Green target zone (bottom-right region)
- 'ducks': Duck collection area (top-left region)
- 'red': Red target zone (top-right region)

**Usage Examples:**
- Single goal: 'balls'
- Multiple goals: 'balls green red ducks'
- Queue addition: Goals can be added during active navigation

**Performance Characteristics:**
- choosing shortest path always
- Goal detection: 0.8m threshold with polygon validation
- Obstacle avoidance: 0.3m detection threshold with LiDAR


Personal Notes: 
- I have taken the help of augment code to complete the assignments. 
- The ideas are completely driven by myself.

Author: Ahmed Maruf, SID: 250046920
Course: Robotics and Autonomous Systems
"""

from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range
from navigation_controller import NavigationController


def initialize_system():
    """
    Initialize the TiaGo robot navigation system.

    Returns:
        tuple: (robot, nav_controller, keyboard) - Initialized system components
    """
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    nav_controller = NavigationController(robot, timestep)
    keyboard = KeyboardReader(timestep)

    return robot, nav_controller, keyboard


def print_system_info():
    """Print system initialization information and usage instructions."""
    print("TiaGo Advanced Multi-Goal Navigation System Initialized")
    print("=" * 65)
    print("Supported targets: 'balls', 'green', 'ducks', 'red'")
    print("Navigation method: PDDL-based optimal path planning")
    print("Network topology: 11-node strategic waypoint system")
    print("Features: Goal queue, obstacle avoidance, progress tracking")
    print("\n Usage Instructions:")
    print("  • Single goal: Type 'balls' and press Enter")
    print("  • Multiple goals: Type 'balls green red' and press Enter")
    print("  • Queue addition: Type goals anytime during navigation")
    print("  • Expected output: 'Goal reached! Total distance covered: X.XX meters'")
    print("=" * 65)


# Initialize system components
robot, nav_controller, keyboard = initialize_system()
timestep = int(robot.getBasicTimeStep())  # Get timestep for main loop
print_system_info()


def process_user_commands(command, nav_controller):
    """
    Process user input commands for goal navigation.

    Args:
        command (str): User input command containing goal names
        nav_controller (NavigationController): Navigation controller instance
    """
    goals = command.strip().lower().split()

    # Filter valid goals
    valid_goals = [goal for goal in goals if goal in ['balls', 'green', 'ducks', 'red']]
    invalid_goals = [goal for goal in goals if goal not in ['balls', 'green', 'ducks', 'red']]

    if invalid_goals:
        print(f"Invalid goals ignored: {invalid_goals}")

    if valid_goals:
        if nav_controller.state in ["IDLE", "GOAL_REACHED"]:
            success = nav_controller.start_queue_navigation(valid_goals)
            if not success:
                print(f"Failed to start queue navigation")
        else: 
            nav_controller.add_multiple_goals_to_queue(valid_goals)
    else:
        print(f"No valid goals found. Supported: 'balls', 'green', 'ducks', 'red'")


def monitor_goal_detection(nav_controller):
    """
    Monitor and report goal proximity detection.

    Args:
        nav_controller (NavigationController): Navigation controller instance
    """
    robot_pos = nav_controller.get_robot_position()
    nearby_goals = get_goals_in_range(*robot_pos)

    # Only report goal detection once per goal and only if we're navigating to it
    if nearby_goals and not nav_controller.goal_reached:
        current_goal = nav_controller.current_target_goal
        for goal in nearby_goals:
            if goal in ['balls', 'green', 'ducks', 'red'] and goal == current_goal:
                # Only print once per detection cycle (use a simple state check)
                if not hasattr(nav_controller, '_last_detected_goal') or nav_controller._last_detected_goal != goal:
                    print(f'Robot detected near {goal} target!')
                    nav_controller._last_detected_goal = goal


def report_system_status(nav_controller, timestep):
    """
    Generate periodic system status reports.

    Args:
        nav_controller (NavigationController): Navigation controller instance
        timestep (int): Simulation timestep for timing calculations
    """
    if nav_controller.robot.getTime() % 10 < timestep / 1000.0:
        status = nav_controller.get_status()

        if status['state'] != "IDLE":
            status_msg = f"Status: {status['state']}"

            if status['current_goal']:
                status_msg += f" | Goal: {status['current_goal']}"

            if status['current_target']:
                status_msg += f" | Target: {status['current_target']}"

            status_msg += f" | Distance: {status['distance_covered']:.2f}m"

            if status['queue'] or status['total_goals'] > 1:
                status_msg += f" | Progress: {status['progress']}"
                if status['queue']:
                    status_msg += f" | Queue: {status['queue']}"

            print(status_msg)


# ============================================================================
# MAIN CONTROL LOOP - Advanced Multi-Goal Navigation System
# ============================================================================
# This loop implements the primary control logic for the multi-goal
# navigation system, integrating user input processing, PDDL path planning,
# and real-time navigation control with comprehensive error handling.

while robot.step(timestep) != -1:
    # ========================================================================
    # COMMAND PROCESSING
    # ========================================================================

    command = keyboard.get_command()
    if command is not None:
        print(f'Command received: {command}')
        process_user_commands(command, nav_controller)

    # ========================================================================
    # NAVIGATION CONTROL
    # ========================================================================
    nav_controller.update()

    # ========================================================================
    # GOAL DETECTION AND VALIDATION
    # ========================================================================
    monitor_goal_detection(nav_controller)

    # ========================================================================
    # SYSTEM MONITORING AND DIAGNOSTICS
    # ========================================================================
    report_system_status(nav_controller, timestep)
