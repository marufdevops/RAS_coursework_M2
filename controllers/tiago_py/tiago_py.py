#
# Mission 2: Navigation with Reactive Obstacle Avoidance
# Week 1 Implementation: Basic Direct Navigation
#
# APPROACH RATIONALE:
# This implementation uses a simple but effective direct navigation approach:
# 1. SIMPLE DIRECT NAVIGATION: Robot turns towards target, then drives straight
#    - Rationale: Most efficient path when obstacles are not present
#    - Uses basic trigonometry (atan2) for angle calculation
#    - Proportional control for smooth turning without overshooting
#
# 2. STATE MACHINE CONTROL: Clear behavior states for reliable operation
#    - States: waiting -> turning -> driving -> completed
#    - Rationale: Predictable behavior, easy to debug and extend
#    - Each state has specific purpose and clear transitions
#
# 3. SENSOR INTEGRATION: GPS for positioning, compass for orientation
#    - Rationale: GPS provides accurate global positioning
#    - Compass enables precise heading control for efficient turning
#    - Combined sensor data ensures reliable navigation
#
# TARGET LOCATIONS:
#  - Red box: (3.685, 3.01) - wooden box with red top
#  - Green box: (3.685, -3.94) - wooden box with green top
#  - Ducks: (-3.08, 3.64) - container with rubber ducks
#  - Balls: (-2.81, -4.17) - container with soccer balls
#
# SUCCESS CRITERIA: Robot reaches within 0.8m of target (detected by goalchecker)
#
# CURRENT LIMITATIONS (to be addressed in Week 2):
# - No obstacle avoidance (will crash into walls/robots)
# - Single target only (no queue management yet)
# - No path planning around obstacles
#
# NEXT STEPS:
# - Week 2: Add LiDAR-based obstacle avoidance
# - Week 3: Implement task queue for multiple commands
# - Week 4: Polish and optimize for maximum marks
#

import math
from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range

robot = Robot()

timestep = int(robot.getBasicTimeStep())


l_motor = robot.getDevice("wheel_left_joint")
r_motor = robot.getDevice("wheel_right_joint")
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
l_motor.setPosition(math.inf)
r_motor.setPosition(math.inf)
l_motor.setVelocity(0)
r_motor.setVelocity(0)

keyboard = KeyboardReader(timestep)

# Target locations mapping - extracted from goalchecker.py polygons
TARGET_LOCATIONS = {
    'red': (3.685, 3.01),      # Center of red box area
    'green': (3.685, -3.94),   # Center of green box area
    'ducks': (-3.08, 3.64),    # Center of ducks container area
    'balls': (-2.81, -4.17)    # Center of balls container area
}

def get_robot_position():
    """
    Get current robot position from GPS sensor
    Rationale: GPS provides accurate global positioning needed for navigation
    Returns: (x, y) coordinates as tuple
    """
    gps_values = gps.getValues()
    x = gps_values[0]  # X coordinate (east-west)
    y = gps_values[1]  # Y coordinate (north-south)
    return (x, y)

def get_robot_heading():
    """
    Get current robot heading from compass sensor
    Rationale: Compass provides orientation needed for turning towards targets
    Returns: angle in radians (-pi to pi)
    """
    compass_values = compass.getValues()
    # Calculate heading angle from compass readings
    heading = math.atan2(compass_values[0], -compass_values[2])
    return heading

def navigate_to_target(current_pos, target_name):
    """
    Calculate navigation information to reach a target
    Rationale: Simple direct navigation is most efficient when path is clear

    Args:
        current_pos: (x, y) tuple of current robot position
        target_name: string name of target ('red', 'green', 'ducks', 'balls')

    Returns:
        tuple: (target_angle, distance, target_pos) or None if invalid target
    """
    if target_name not in TARGET_LOCATIONS:
        print(f"Unknown target: {target_name}")
        return None

    target_pos = TARGET_LOCATIONS[target_name]

    # Calculate direction vector to target
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]

    # Calculate angle to target (in radians)
    target_angle = math.atan2(dy, dx)

    # Calculate distance to target
    distance = math.sqrt(dx*dx + dy*dy)

    return (target_angle, distance, target_pos)

def angle_difference(angle1, angle2):
    """
    Calculate the shortest angular difference between two angles
    Rationale: Needed for efficient turning - always turn the shorter way

    Returns: difference in radians (-pi to pi)
    """
    diff = angle2 - angle1
    # Normalize to [-pi, pi] range
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff

def set_robot_speeds(left_speed, right_speed):
    """
    Set the speeds of left and right wheels
    Rationale: Differential drive control - different wheel speeds create turning

    Args:
        left_speed: speed for left wheel (-6.28 to 6.28 rad/s)
        right_speed: speed for right wheel (-6.28 to 6.28 rad/s)
    """
    # Clamp speeds to safe limits
    max_speed = 3.0
    left_speed = max(-max_speed, min(max_speed, left_speed))
    right_speed = max(-max_speed, min(max_speed, right_speed))

    l_motor.setVelocity(left_speed)
    r_motor.setVelocity(right_speed)

def turn_towards_angle(current_heading, target_angle, turn_speed=1.0):
    """
    Turn robot towards a target angle
    Rationale: Proportional control for smooth turning without overshooting

    Args:
        current_heading: current robot heading in radians
        target_angle: desired heading in radians
        turn_speed: turning speed multiplier

    Returns:
        True if close enough to target angle, False if still turning
    """
    angle_error = angle_difference(current_heading, target_angle)

    # If close enough to target angle, stop turning
    if abs(angle_error) < 0.1:  # ~6 degrees tolerance
        set_robot_speeds(0, 0)
        return True

    # Turn towards target - proportional control
    turn_rate = angle_error * 2.0 * turn_speed  # Proportional gain
    set_robot_speeds(-turn_rate, turn_rate)
    return False

def drive_forward(speed=1.0):
    """
    Drive robot forward at specified speed
    Rationale: Simple forward motion for direct navigation to target

    Args:
        speed: forward speed (0 to 3.0)
    """
    set_robot_speeds(speed, speed)

def stop_robot():
    """
    Stop all robot movement
    Rationale: Safety function and used when reaching targets
    """
    set_robot_speeds(0, 0)

# Simple navigation state machine
current_target = None
navigation_state = "waiting"  # States: "waiting", "turning", "driving", "completed"
step_counter = 0  # For debug output

print("=== Week 1: Basic Navigation System Started ===")
print("Available commands: red, green, ducks, balls")
print("Click in 3D view and type command + Enter")

while (robot.step(timestep) != -1):
    step_counter += 1
    # Get current robot state
    current_pos = get_robot_position()
    current_heading = get_robot_heading()

    # Handle keyboard commands
    command = keyboard.get_command()
    if command is not None:
        print(f'Got command: {command}')
        if command in TARGET_LOCATIONS:
            current_target = command
            navigation_state = "turning"
            print(f'Starting navigation to {command}')

    # Navigation state machine
    if navigation_state == "waiting":
        stop_robot()

    elif navigation_state == "turning" and current_target:
        # Calculate navigation to target
        nav_info = navigate_to_target(current_pos, current_target)
        if nav_info:
            target_angle, distance, target_pos = nav_info
            print(f'Distance to {current_target}: {distance:.2f}m')

            # Turn towards target
            if turn_towards_angle(current_heading, target_angle):
                print(f'Turned towards {current_target}, now driving')
                navigation_state = "driving"

    elif navigation_state == "driving" and current_target:
        # Check if we've reached the target
        try:
            goals_in_range = get_goals_in_range(*current_pos)
            if current_target in goals_in_range:
                print(f'Reached {current_target}!')
                stop_robot()
                navigation_state = "completed"
                current_target = None
            else:
                # Keep driving forward
                drive_forward(1.5)
        except Exception as e:
            print(f"Goal checking error: {e}")
            # Fallback: check distance manually
            nav_info = navigate_to_target(current_pos, current_target)
            if nav_info and nav_info[1] < 0.8:  # Within 0.8m threshold
                print(f'Reached {current_target} (fallback detection)!')
                stop_robot()
                navigation_state = "completed"
                current_target = None
            else:
                drive_forward(1.5)

    elif navigation_state == "completed":
        print("Mission completed! Waiting for next command...")
        navigation_state = "waiting"

    # Debug output every 5 seconds
    if step_counter % 500 == 0:
        print(f"Status: {navigation_state}, Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f}), Target: {current_target}")
    


