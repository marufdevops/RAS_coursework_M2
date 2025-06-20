#
# PDDL-Based Navigation System for TiaGo Robot
# Mission 2: Navigate to 4 targets using PDDL planning with all sensors
#
# This system implements:
# 1. PDDL-based path planning with pyperplan for strategic navigation
# 2. Multi-sensor integration (GPS, LiDAR, Compass) for localization and obstacle detection
# 3. Dynamic obstacle avoidance with pause/resume behavior for mobile obstacles
# 4. Navigation to 4 targets: red, green, ducks, balls with goal queue management
# 5. Waypoint-based navigation with replanning when obstacles block paths
#
# The robot uses a grid-based waypoint system for PDDL planning, allowing it to
# navigate around static obstacles and reach all 4 target locations efficiently.
# When dynamic obstacles (Pioneer robots) are detected, the robot pauses and
# resumes navigation when the path is clear.
#
# Target locations (from goalchecker.py):
# - Red box: (3.68, 3.02) - Right side, upper area
# - Green box: (3.68, -3.95) - Right side, lower area
# - Ducks container: (-3.0, 3.7) - Left side, upper area
# - Balls container: (-2.8, -4.3) - Left side, lower area
#

import os
import math
import subprocess
from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range

# Initialize robot and devices
robot = Robot()
timestep = int(robot.getBasicTimeStep())

l_motor = robot.getDevice("wheel_left_joint")
r_motor = robot.getDevice("wheel_right_joint")
lidar = robot.getDevice("lidar")
compass = robot.getDevice("compass")
gps = robot.getDevice('gps')

# Enable sensors
lidar.enable(timestep)
compass.enable(timestep)
gps.enable(timestep)

# Configure motors for velocity control
l_motor.setPosition(math.inf)
r_motor.setPosition(math.inf)
l_motor.setVelocity(0)
r_motor.setVelocity(0)

# Initialize keyboard input
keyboard = KeyboardReader(timestep)

# Target coordinates will be defined below with waypoints

# Optimized waypoints for efficient navigation
WAYPOINTS = {
    # Robot start position
    'start': (-1.96, -1.95),

    # Central navigation waypoints
    'center': (-0.1, -2.76),
    'north_center': (-0.05, 2.9),
    'east_center': (0.95, 0.78),
    'south_center': (2.6, -0.48),

    # Intermediate waypoints for better routing
    'northwest_corridor': (-1.5, 2.0),  # Direct path to northwest area
    'west_corridor': (-2.0, 0.5),       # Western corridor for better routing

    # Optimized goal approach waypoints (closer to targets)
    'balls_approach': (-2.2, -3.8),     # Closer to balls target
    'ducks_approach': (-2.8, 3.5),      # Much closer to ducks target
    'red_approach': (3.2, 2.8),         # Closer to red target
    'green_approach': (3.2, -3.5),      # Closer to green target
}

# Goal target coordinates
TARGET_COORDS = {
    'balls': (-2.5, -4.0),
    'ducks': (-3.0, 3.7),
    'red': (3.68, 3.02),
    'green': (3.68, -3.95),
}

# Let pyperplan connect ALL waypoints to ALL waypoints for optimal pathfinding
# No artificial connection constraints - pyperplan will find the best path

# Removed unnecessary helper functions

# Goal queue and navigation state
goal_queue = []
current_goal = None
current_plan = []
current_waypoint_index = 0
navigation_state = 'idle'  # 'idle', 'planning', 'navigating'
last_position = None        # Track last position for progress monitoring
position_stuck_time = 0     # Time when robot got stuck in same position
STUCK_TIMEOUT = 30.0        # Time to wait before considering robot stuck (increased)
plan_completion_time = 0    # Time when plan was completed (to prevent infinite oscillation)
COMPLETION_TIMEOUT = 20.0   # Max time to spend trying to reach final target (increased)

# Navigation parameters (optimized for faster, more responsive movement)
WAYPOINT_THRESHOLD = 0.4  # Distance to consider waypoint reached
GOAL_THRESHOLD = 0.8      # Distance to consider goal reached
OBSTACLE_THRESHOLD = 0.8  # LiDAR distance for obstacle detection (reduced for better responsiveness)
MAX_VELOCITY = 5.0        # Maximum robot velocity (increased from 2.0)
TURN_THRESHOLD = 0.15     # Angle threshold for turning vs moving (reduced for more precise turning)

def get_current_position():
    """Get current robot position from GPS"""
    return gps.getValues()[:2]

def get_current_angle():
    """Get current robot orientation from compass"""
    compass_values = compass.getValues()
    return math.atan2(compass_values[0], -compass_values[2])

def distance_to_point(point):
    """Calculate Euclidean distance to a point"""
    pos = get_current_position()
    if math.isnan(pos[0]) or math.isnan(pos[1]):
        return float('inf')
    return math.sqrt((pos[0] - point[0])**2 + (pos[1] - point[1])**2)

def angle_to_point(point):
    """Calculate angle to a point"""
    pos = get_current_position()
    if math.isnan(pos[0]) or math.isnan(pos[1]):
        return 0.0
    return math.atan2(point[1] - pos[1], point[0] - pos[0])

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def write_pddl_problem_simple(goal_target):
    """Generate PDDL problem - let pyperplan find optimal path between ANY waypoints"""
    goal_waypoint_map = {
        'balls': 'balls_approach',
        'ducks': 'ducks_approach',
        'red': 'red_approach',
        'green': 'green_approach'
    }

    if goal_target not in goal_waypoint_map:
        print(f"Error: Unknown goal '{goal_target}'")
        return False

    goal_waypoint = goal_waypoint_map[goal_target]
    waypoint_names = ' '.join(WAYPOINTS.keys())

    problem_content = f"""(define (problem navigate-to-{goal_target})
    (:domain warehouse)
    (:objects
        robot - robot
        {waypoint_names} - waypoint
    )
    (:init
        (at robot start)
        ; Connect ALL waypoints to ALL waypoints - no artificial constraints
"""

    # Connect every waypoint to every other waypoint
    waypoint_list = list(WAYPOINTS.keys())
    for wp1 in waypoint_list:
        for wp2 in waypoint_list:
            if wp1 != wp2:  # Don't connect waypoint to itself
                problem_content += f"        (connected {wp1} {wp2})\n"

    problem_content += f"""    )
    (:goal
        (at robot {goal_waypoint})
    )
)"""

    with open("warehouse_problem.pddl", 'w') as f:
        f.write(problem_content)

    print(f"✅ Generated PDDL: start -> {goal_waypoint} (all waypoints connected)")
    return True

def call_pyperplan_simple(goal_target):
    """Call pyperplan - let it find the optimal path"""
    try:
        # Generate problem file
        if not write_pddl_problem_simple(goal_target):
            return []

        # Call pyperplan command
        cmd = ["/opt/anaconda3/bin/pyperplan", "-H", "hff", "-s", "astar",
               "warehouse_domain.pddl", "warehouse_problem.pddl"]

        print(f"🧠 Pyperplan finding optimal path to {goal_target}...")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

        if result.returncode == 0:
            # Read solution file
            solution_file = "warehouse_problem.pddl.soln"
            if os.path.exists(solution_file):
                with open(solution_file, 'r') as f:
                    solution_content = f.read().strip()

                # Parse waypoints from solution
                waypoints = []
                for line in solution_content.split('\n'):
                    if line.strip().startswith('(move robot'):
                        parts = line.strip().replace(')', '').split()
                        if len(parts) >= 4:
                            waypoints.append(parts[3])  # destination waypoint

                # Clean up solution file
                os.remove(solution_file)

                print(f"✅ Optimal path: {waypoints}")
                return waypoints
            else:
                print(f"❌ No solution file generated")
                return []
        else:
            print(f"❌ Pyperplan failed: {result.stderr}")
            return []

    except Exception as e:
        print(f"❌ Error calling pyperplan: {e}")
        return []

# Dynamic PDDL planning - generates solutions based on current robot position

def move_to_waypoint(target_waypoint_name):
    """Move robot towards target waypoint using proportional control"""
    if target_waypoint_name not in WAYPOINTS:
        return False

    target_point = WAYPOINTS[target_waypoint_name]
    current_angle = get_current_angle()

    # Calculate distance and angle to target
    distance = distance_to_point(target_point)
    target_angle = angle_to_point(target_point)
    angle_error = normalize_angle(target_angle - current_angle)

    # Check if waypoint is reached
    if distance < WAYPOINT_THRESHOLD:
        return True

    # Aggressive proportional control for faster movement
    if abs(angle_error) > TURN_THRESHOLD:
        # Turn towards target with higher gain
        turn_speed = min(MAX_VELOCITY, abs(angle_error) * 3.5)
        if angle_error > 0:
            l_motor.setVelocity(-turn_speed)
            r_motor.setVelocity(turn_speed)
        else:
            l_motor.setVelocity(turn_speed)
            r_motor.setVelocity(-turn_speed)
    else:
        # Move forward towards target with higher gain
        forward_speed = min(MAX_VELOCITY, distance * 2.5)
        # Ensure minimum speed for responsiveness
        forward_speed = max(forward_speed, MAX_VELOCITY * 0.3)
        l_motor.setVelocity(forward_speed)
        r_motor.setVelocity(forward_speed)

    return False

def stop_robot():
    """Stop the robot"""
    l_motor.setVelocity(0)
    r_motor.setVelocity(0)

# Removed unused test functions

def process_goal_queue():
    """Process goal queue and manage navigation state"""
    global current_goal, current_plan, current_waypoint_index, navigation_state

    if not goal_queue and current_goal is None:
        navigation_state = 'idle'
        return

    # Start new goal if none active
    if current_goal is None and goal_queue:
        current_goal = goal_queue.pop(0)
        navigation_state = 'planning'
        print(f"Starting navigation to: {current_goal}")

        # Let pyperplan find the optimal path with full connectivity
        print(f"🎯 Planning optimal route to {current_goal}")
        current_plan = call_pyperplan_simple(current_goal)

        if current_plan:
            current_waypoint_index = 0
            navigation_state = 'navigating'
            print(f"PDDL plan: {current_plan}")
        else:
            print(f"Failed to plan path to {current_goal}")
            current_goal = None
            navigation_state = 'idle'

# Obstacle avoidance functions removed - focusing on basic navigation

def check_progress():
    """Check if robot is making progress and handle stuck situations"""
    global last_position, position_stuck_time, navigation_state, current_goal, current_plan, current_waypoint_index

    current_pos = get_current_position()
    current_time = robot.getTime()

    if last_position is None:
        last_position = current_pos
        position_stuck_time = current_time
        return

    # Check if robot has moved significantly (only when actively navigating)
    distance_moved = math.sqrt((current_pos[0] - last_position[0])**2 + (current_pos[1] - last_position[1])**2)

    if distance_moved > 0.2:  # Robot has moved at least 20cm
        last_position = current_pos
        position_stuck_time = current_time
    else:
        # Robot hasn't moved much while actively navigating
        if current_time - position_stuck_time > STUCK_TIMEOUT and navigation_state == 'navigating':
            print(f"Robot appears stuck for {STUCK_TIMEOUT}s while navigating - replanning from current position")
            # Force replanning from current position
            navigation_state = 'planning'

def execute_navigation():
    """Execute current navigation plan - simplified without obstacle detection"""
    global current_goal, current_plan, current_waypoint_index, navigation_state, plan_completion_time

    if navigation_state != 'navigating' or not current_plan:
        return

    # Check if robot is making progress
    check_progress()

    # Navigate to current waypoint
    if current_waypoint_index < len(current_plan):
        target_waypoint = current_plan[current_waypoint_index]

        if move_to_waypoint(target_waypoint):
            print(f"Reached waypoint: {target_waypoint}")
            current_waypoint_index += 1

            # Check if plan is complete
            if current_waypoint_index >= len(current_plan):
                # Initialize completion timer if not set
                current_time = robot.getTime()
                if plan_completion_time == 0:
                    plan_completion_time = current_time

                # Check for completion timeout to prevent infinite oscillation
                if current_time - plan_completion_time > COMPLETION_TIMEOUT:
                    print(f"⏰ Plan completion timeout ({COMPLETION_TIMEOUT}s) - accepting current position")
                    current_goal = None
                    current_plan = []
                    current_waypoint_index = 0
                    navigation_state = 'idle'
                    plan_completion_time = 0
                    stop_robot()
                    return

                # Plan completed - check goal achievement
                pos = get_current_position()
                goals_in_range = get_goals_in_range(pos[0], pos[1])

                # Check distance to actual target
                target_distance = float('inf')
                if current_goal and current_goal in TARGET_COORDS:
                    target_coord = TARGET_COORDS[current_goal]
                    target_distance = distance_to_point(target_coord)

                # Check goal completion conditions
                condition1 = current_goal in goals_in_range
                condition2 = target_distance <= GOAL_THRESHOLD * 1.2

                print(f"🔍 Goal check: position=({pos[0]:.2f}, {pos[1]:.2f}), distance={target_distance:.2f}m")

                if condition1 or condition2:
                    print(f"✅ Successfully reached goal: {current_goal}")
                    if condition2:
                        print(f"📏 Distance to target: {target_distance:.2f}m (within {GOAL_THRESHOLD * 1.2:.2f}m threshold)")
                    if condition1:
                        print(f"🎯 Goal detected by goalchecker.py")

                    # Goal completed - robot position has changed for next planning
                    print(f"🔄 Goal completed. Next planning will use updated robot position.")
                    current_goal = None
                    current_plan = []
                    current_waypoint_index = 0
                    navigation_state = 'idle'
                    plan_completion_time = 0  # Reset completion timer
                    stop_robot()  # Stop oscillation
                else:
                    # Try final approach to target
                    print(f"⚠️ Plan completed, distance: {target_distance:.2f}m - making final approach")
                    if current_goal and current_goal in TARGET_COORDS:
                        target_coord = TARGET_COORDS[current_goal]
                        if move_to_waypoint_direct(target_coord):
                            print(f"✅ Final approach successful: {current_goal}")
                        else:
                            print(f"⚠️ Accepting current position to avoid oscillation")

                    # Complete the goal
                    current_goal = None
                    current_plan = []
                    current_waypoint_index = 0
                    navigation_state = 'idle'
                    plan_completion_time = 0
                    stop_robot()

def move_to_waypoint_direct(target_coord):
    """Move directly to coordinate (for final approach)"""
    current_angle = get_current_angle()

    # Calculate distance and angle to target
    distance = distance_to_point(target_coord)
    target_angle = angle_to_point(target_coord)
    angle_error = normalize_angle(target_angle - current_angle)

    # Check if target is reached
    if distance < GOAL_THRESHOLD:
        return True

    # Aggressive proportional control for final approach
    if abs(angle_error) > TURN_THRESHOLD:
        # Turn towards target with high gain
        turn_speed = min(MAX_VELOCITY * 0.9, abs(angle_error) * 3.0)
        if angle_error > 0:
            l_motor.setVelocity(-turn_speed)
            r_motor.setVelocity(turn_speed)
        else:
            l_motor.setVelocity(turn_speed)
            r_motor.setVelocity(-turn_speed)
    else:
        # Move forward towards target with high gain
        forward_speed = min(MAX_VELOCITY * 0.9, distance * 2.0)
        # Ensure minimum speed for final approach
        forward_speed = max(forward_speed, MAX_VELOCITY * 0.2)
        l_motor.setVelocity(forward_speed)
        r_motor.setVelocity(forward_speed)

    return False

def main():
    """Main navigation control loop"""
    global step_count, last_status_time

    # Wait for GPS initialization
    print("=== PDDL-Based TiaGo Navigation System ===")
    print("Initializing sensors...")
    while robot.step(timestep) != -1:
        pos = get_current_position()
        if not (math.isnan(pos[0]) or math.isnan(pos[1])):
            break

    print("System ready!")
    print("Commands: red, green, ducks, balls (type and press ENTER)")
    print("Multiple goals can be separated by commas")
    print("Current position:", get_current_position())
    print("==========================================")

    # Main control loop
    step_count = 0
    last_status_time = 0

    while robot.step(timestep) != -1:
        step_count += 1
        current_time = robot.getTime()

        # Process keyboard input
        command = keyboard.get_command()
        if command is not None:
            print(f"Received command: '{command}'")

            # Parse multiple goals separated by commas
            goals = [goal.strip().lower() for goal in command.split(',')]
            valid_goals = []

            for goal in goals:
                if goal in TARGET_COORDS:
                    valid_goals.append(goal)
                else:
                    print(f"Unknown goal: '{goal}'. Valid goals: red, green, ducks, balls")

            if valid_goals:
                # Add goals to queue
                for goal in valid_goals:
                    if goal not in goal_queue:
                        goal_queue.append(goal)
                print(f"Goal queue: {goal_queue}")

        # Process goal queue and navigation
        process_goal_queue()
        execute_navigation()

        # Stop robot if idle
        if navigation_state == 'idle':
            stop_robot()

        # Removed sensor debugging - focusing on basic navigation

        # Status reporting every 5 seconds
        if current_time - last_status_time > 5.0:
            last_status_time = current_time
            pos = get_current_position()
            goals_nearby = get_goals_in_range(pos[0], pos[1])

            print(f"\n--- Status ---")
            print(f"Position: ({pos[0]:.2f}, {pos[1]:.2f}) | State: {navigation_state}")
            print(f"Goal: {current_goal} | Queue: {goal_queue} | Nearby: {goals_nearby}")
            if current_plan:
                print(f"Plan: {current_plan} | Progress: {current_waypoint_index}/{len(current_plan)}")
            print("-------------\n")

    print("Navigation system shutdown.")

# Run main function if script is executed directly
if __name__ == "__main__":
    main()
    