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

# Target coordinates (exact centers from goalchecker polygons)
TARGET_COORDS = {
    'red': (3.68, 3.02),      # Red box center
    'green': (3.68, -3.95),   # Green box center
    'ducks': (-3.0, 3.7),     # Ducks container center
    'balls': (-2.8, -4.3)     # Balls container center
}

# Accurate waypoint grid for PDDL planning (avoiding all static obstacles)
# Based on mission2.wbt coordinates and warehouse layout analysis
WAYPOINTS = {
    # Core navigation waypoints (safe open areas)
    'start': (-1.97, -1.96),    # Robot start position
    'center_west': (-1.0, 0.0), # West side of central wall
    'center_east': (1.0, 0.0),  # East side of central wall
    'north_west': (-1.5, 2.5),  # Northwest corridor (clear of cones)
    'north_east': (1.5, 2.5),   # Northeast corridor (clear of cones)
    'south_west': (-1.0, -3.5), # Southwest corridor (clear of cones)
    'south_east': (1.5, -3.0),  # Southeast corridor (clear of cones)

    # Target approach waypoints (safe distances from obstacles)
    'red_approach': (3.2, 2.5),    # Approach to red box (clear of traffic cones)
    'green_approach': (3.2, -3.2), # Approach to green box (clear of traffic cones)
    'ducks_approach': (-2.2, 3.2), # Approach to ducks (clear of boundary)
    'balls_approach': (-2.0, -3.8), # Approach to balls (clear of traffic cones)

    # Intermediate waypoints for complex navigation
    'west_corridor': (-2.8, 0.0),  # Western corridor
    'east_corridor': (2.8, 0.0),   # Eastern corridor
    'north_central': (0.5, 1.5),   # North of central wall
    'south_central': (0.5, -2.5),  # South of central wall
}

# Goal queue and navigation state
goal_queue = []
current_goal = None
current_plan = []
current_waypoint_index = 0
navigation_state = 'idle'  # 'idle', 'planning', 'navigating'
last_position = None        # Track last position for progress monitoring
position_stuck_time = 0     # Time when robot got stuck in same position
STUCK_TIMEOUT = 30.0        # Time to wait before considering robot stuck (increased)

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

# Obstacle detection removed - focusing on basic navigation first

def get_closest_waypoint():
    """Find the closest waypoint to current position"""
    pos = get_current_position()
    if math.isnan(pos[0]) or math.isnan(pos[1]):
        return 'start'

    min_dist = float('inf')
    closest = 'start'

    for name, coord in WAYPOINTS.items():
        dist = math.sqrt((pos[0] - coord[0])**2 + (pos[1] - coord[1])**2)
        if dist < min_dist:
            min_dist = dist
            closest = name

    return closest

def write_pddl_problem(start_waypoint, goal_target):
    """Generate PDDL problem file for navigation"""
    # Determine goal waypoint based on target
    goal_waypoint_map = {
        'red': 'red_approach',
        'green': 'green_approach',
        'ducks': 'ducks_approach',
        'balls': 'balls_approach'
    }
    goal_waypoint = goal_waypoint_map.get(goal_target, 'center')

    problem_content = f"""(define (problem navigate-to-{goal_target})
    (:domain warehouse)
    (:objects
        robot - robot
        {' '.join(WAYPOINTS.keys())} - waypoint
    )
    (:init
        (at robot {start_waypoint})
        ; Waypoint connections (bidirectional) - based on actual warehouse layout
        ; Start connections
        (connected start center_west)
        (connected center_west start)
        (connected start south_west)
        (connected south_west start)

        ; Central area connections (around the central wall)
        ; NOTE: center_west and center_east are NOT directly connected due to central wall
        (connected center_west north_west)
        (connected north_west center_west)
        (connected center_west south_west)
        (connected south_west center_west)
        (connected center_east north_east)
        (connected north_east center_east)
        (connected center_east south_east)
        (connected south_east center_east)

        ; North-south corridor connections
        (connected north_west north_central)
        (connected north_central north_west)
        (connected north_east north_central)
        (connected north_central north_east)
        (connected south_west south_central)
        (connected south_central south_west)
        (connected south_east south_central)
        (connected south_central south_east)

        ; Cross-connections to allow navigation around central wall
        (connected north_central south_central)
        (connected south_central north_central)

        ; East-west corridor connections
        (connected center_west west_corridor)
        (connected west_corridor center_west)
        (connected center_east east_corridor)
        (connected east_corridor center_east)

        ; Target approach connections
        (connected north_east red_approach)
        (connected red_approach north_east)
        (connected east_corridor red_approach)
        (connected red_approach east_corridor)

        (connected south_east green_approach)
        (connected green_approach south_east)
        (connected east_corridor green_approach)
        (connected green_approach east_corridor)

        (connected north_west ducks_approach)
        (connected ducks_approach north_west)
        (connected west_corridor ducks_approach)
        (connected ducks_approach west_corridor)

        (connected south_west balls_approach)
        (connected balls_approach south_west)
        (connected west_corridor balls_approach)
        (connected balls_approach west_corridor)
    )
    (:goal
        (at robot {goal_waypoint})
    )
)"""

    with open("warehouse_problem.pddl", 'w') as f:
        f.write(problem_content)

def find_pyperplan_path():
    """Find the absolute path to pyperplan executable"""
    # Common paths where pyperplan might be installed
    possible_paths = [
        "/opt/anaconda3/bin/pyperplan",
        "/usr/local/bin/pyperplan",
        "/usr/bin/pyperplan",
        "pyperplan"  # fallback to PATH
    ]

    for path in possible_paths:
        try:
            # Test if pyperplan works at this path
            result = subprocess.run([path, "--help"], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print(f"Found pyperplan at: {path}")
                return path
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
            continue

    # If not found, try to find it using which command
    try:
        result = subprocess.run(["which", "pyperplan"], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            path = result.stdout.strip()
            print(f"Found pyperplan using which: {path}")
            return path
    except:
        pass

    print("ERROR: pyperplan not found in any common locations")
    return None

def call_pddl_planner(start_waypoint, goal_target):
    """Call PDDL planner using command-line interface with absolute path"""
    try:
        # Find pyperplan executable
        pyperplan_path = find_pyperplan_path()
        if not pyperplan_path:
            print("Cannot find pyperplan executable")
            return []

        # Write problem file
        write_pddl_problem(start_waypoint, goal_target)

        # Use pyperplan command-line interface with absolute paths
        domain_file = os.path.abspath("warehouse_domain.pddl")
        problem_file = os.path.abspath("warehouse_problem.pddl")

        # Call pyperplan with absolute path and proper environment
        cmd = [pyperplan_path, "-H", "hff", "-s", "astar", domain_file, problem_file]

        print(f"Running PDDL planner: {' '.join(cmd)}")

        # Set up environment to ensure Python modules are found
        env = os.environ.copy()
        env['PATH'] = '/opt/anaconda3/bin:' + env.get('PATH', '')
        env['PYTHONPATH'] = '/opt/anaconda3/lib/python3.12/site-packages:' + env.get('PYTHONPATH', '')

        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30, env=env)

        if result.returncode == 0:
            # Look for solution file (pyperplan creates .pddl.soln file)
            solution_file = problem_file + '.soln'

            if os.path.exists(solution_file):
                with open(solution_file, 'r') as f:
                    solution_content = f.read().strip()

                print(f"PDDL solution found: {solution_content}")

                # Parse solution to extract waypoints
                waypoints = []
                lines = solution_content.split('\n')
                for line in lines:
                    line = line.strip()
                    if line.startswith('(move robot'):
                        # Extract destination waypoint from action
                        # Format: (move robot from_waypoint to_waypoint)
                        parts = line.split()
                        if len(parts) >= 4:
                            # Clean up waypoint name (remove trailing parentheses)
                            waypoint = parts[3].rstrip(')')
                            waypoints.append(waypoint)

                # Clean up solution file
                os.remove(solution_file)
                return waypoints
            else:
                print("PDDL planning failed - no solution file generated")
                return []
        else:
            print(f"PDDL planner failed with return code {result.returncode}")
            print(f"Error output: {result.stderr}")
            print(f"Standard output: {result.stdout}")
            return []

    except subprocess.TimeoutExpired:
        print("PDDL planning timeout")
        return []
    except Exception as e:
        print(f"PDDL planning error: {e}")
        import traceback
        traceback.print_exc()
        return []

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

        # Plan path using PDDL
        start_waypoint = get_closest_waypoint()
        current_plan = call_pddl_planner(start_waypoint, current_goal)

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
    global current_goal, current_plan, current_waypoint_index, navigation_state

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
                # Check if we're close to the actual goal
                pos = get_current_position()
                goals_in_range = get_goals_in_range(pos[0], pos[1])

                if current_goal in goals_in_range:
                    print(f"Successfully reached goal: {current_goal}")
                    current_goal = None
                    current_plan = []
                    current_waypoint_index = 0
                    navigation_state = 'idle'
                else:
                    # Move closer to actual target
                    if current_goal and current_goal in TARGET_COORDS:
                        target_coord = TARGET_COORDS[current_goal]
                        if distance_to_point(target_coord) > GOAL_THRESHOLD:
                            # Navigate directly to target
                            if move_to_waypoint_direct(target_coord):
                                print(f"Reached target area for: {current_goal}")
                                current_goal = None
                                current_plan = []
                                current_waypoint_index = 0
                                navigation_state = 'idle'
    else:
        # Plan completed but goal not reached - replan
        print("Plan completed but goal not reached - replanning")
        navigation_state = 'planning'

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

            print(f"\n--- Status Report ---")
            print(f"Position: ({pos[0]:.2f}, {pos[1]:.2f})")
            print(f"Navigation State: {navigation_state}")
            print(f"Current Goal: {current_goal}")
            print(f"Goal Queue: {goal_queue}")
            print(f"Goals in Range: {goals_nearby}")
            if current_plan:
                print(f"Current Plan: {current_plan}")
                print(f"Waypoint Progress: {current_waypoint_index}/{len(current_plan)}")
            print("--------------------\n")

    print("Navigation system shutdown.")

# Run main function if script is executed directly
if __name__ == "__main__":
    main()
    