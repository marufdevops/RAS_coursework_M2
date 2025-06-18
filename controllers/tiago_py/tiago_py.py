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
# WEEK 2 IMPLEMENTATION: LiDAR-based Reactive Obstacle Avoidance
#
# OBSTACLE AVOIDANCE STRATEGY:
# 1. SECTORED LIDAR ANALYSIS: Divide 200-point LiDAR into 5 directional sectors
#    - Rationale: Enables directional obstacle detection and decision-making
#    - Sectors: front, left, right, front_left, front_right
#    - Each sector reports minimum distance to obstacles
#
# 2. SAFETY-FIRST APPROACH: Configurable safety distance (default 1.5m)
#    - Rationale: Prevents collisions with safety margin for robot dynamics
#    - Three urgency levels: high (<0.8m), medium (<1.2m), low (>1.2m)
#    - Speed adaptation: slower forward, faster turns when obstacles close
#
# 3. INTELLIGENT AVOIDANCE DECISIONS: Balance safety with target progress
#    - Emergency mode: Turn away from closest obstacle immediately
#    - Strategic mode: Consider target direction when choosing avoidance
#    - Rationale: Maintains progress toward goal while ensuring safety
#
# 4. REACTIVE BEHAVIOR INTEGRATION: Seamless integration with navigation
#    - Obstacle avoidance takes priority during driving phase
#    - Maintains existing turn-then-drive state machine structure
#    - Rationale: Preserves proven navigation while adding safety
#
# TECHNICAL IMPLEMENTATION:
# - LiDAR: TiagoLite has 3.5 radian FOV (~200°), Pioneer has 3.14159 radian FOV (180°)
# - Real-time obstacle detection and avoidance decision-making with corrected sector mapping
# - Enhanced target memory system to maintain navigation goals during avoidance
# - Comprehensive status reporting for debugging and demonstration
#
# CURRENT LIMITATIONS (to be addressed in Week 3):
# - Single target only (no queue management yet)
# - Reactive avoidance only (no global path planning)
#
# NEXT STEPS:
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

def get_lidar_data():
    """
    Get current LiDAR sensor readings
    Rationale: LiDAR provides 360° obstacle detection needed for safe navigation

    Returns: list of distance readings or None if error
    - Index 0: Far left side
    - Index middle: Front center
    - Index max: Far right side
    """
    try:
        readings = lidar.getRangeImage()
        return readings if readings else None
    except Exception as e:
        print(f"LiDAR reading error: {e}")
        return None

def analyze_lidar_sectors(lidar_readings):
    """
    Analyze LiDAR data by dividing into directional sectors
    Rationale: Sectored analysis enables directional obstacle detection

    CORRECTED MAPPING FOR TIAGO LIDAR:
    - TiagoLite LiDAR: 3.5 radians FOV (~200°), 200 points, 30m range
    - Scans from approximately -100° to +100° (left to right)
    - Index 0: Far left (-100°), Index 199: Far right (+100°)
    - Front (0°) is at index 100 (middle of array)

    Args:
        lidar_readings: list of distance readings from LiDAR

    Returns:
        dict with keys: 'front', 'left', 'right', 'front_left', 'front_right'
        Each value is the minimum distance in that sector
    """
    if not lidar_readings or len(lidar_readings) == 0:
        return None

    total_points = len(lidar_readings)

    # Calculate angular resolution
    # TiagoLite: 3.5 radians / 200 points = 0.0175 rad/point (~1° per point)
    angular_resolution = 3.5 / total_points if total_points > 0 else 0.0175

    # For 3.5 radian FOV, the scan goes from -1.75 to +1.75 radians
    # Front (0°) is at the center index
    front_center = total_points // 2

    # Define sector widths in terms of indices
    # Front: ±15° around center (±0.26 radians)
    front_half_width = max(5, int(0.26 / angular_resolution))  # At least 5 points
    front_start = front_center - front_half_width
    front_end = front_center + front_half_width

    # Left side: from start to -30° (left of front-left)
    left_start = 0
    left_boundary_angle = -0.52  # -30 degrees in radians
    left_end = max(10, int((1.75 + left_boundary_angle) / angular_resolution))

    # Right side: from +30° to end (right of front-right)
    right_boundary_angle = 0.52  # +30 degrees in radians
    right_start = min(total_points - 10, int((1.75 + right_boundary_angle) / angular_resolution))
    right_end = total_points

    # Front-left: from -30° to front sector
    front_left_start = left_end
    front_left_end = front_start

    # Front-right: from front sector to +30°
    front_right_start = front_end
    front_right_end = right_start

    # Ensure valid ranges
    front_start = max(0, min(front_start, total_points-1))
    front_end = max(front_start+1, min(front_end, total_points))
    left_end = max(1, min(left_end, total_points))
    right_start = max(0, min(right_start, total_points-1))
    front_left_start = max(left_end, 0)
    front_left_end = max(front_left_start+1, min(front_left_end, total_points))
    front_right_start = max(front_end, 0)
    front_right_end = max(front_right_start+1, min(front_right_end, total_points))

    sectors = {
        'front': min(lidar_readings[front_start:front_end]) if front_end > front_start else float('inf'),
        'left': min(lidar_readings[left_start:left_end]) if left_end > left_start else float('inf'),
        'right': min(lidar_readings[right_start:right_end]) if right_end > right_start else float('inf'),
        'front_left': min(lidar_readings[front_left_start:front_left_end]) if front_left_end > front_left_start else float('inf'),
        'front_right': min(lidar_readings[front_right_start:front_right_end]) if front_right_end > front_right_start else float('inf')
    }

    return sectors

def detect_obstacles(lidar_readings, safety_distance=1.5):
    """
    Detect obstacles in different directions around the robot
    Rationale: Safety-first approach with configurable safety distance

    Args:
        lidar_readings: raw LiDAR distance readings
        safety_distance: minimum safe distance to obstacles (meters)

    Returns:
        dict with obstacle detection results:
        - 'obstacles_detected': True if any obstacles within safety distance
        - 'safe_directions': list of directions without obstacles
        - 'closest_obstacle': direction and distance of closest obstacle
        - 'sectors': detailed sector analysis
    """
    sectors = analyze_lidar_sectors(lidar_readings)
    if not sectors:
        return None

    # Check which sectors have obstacles within safety distance
    obstacle_sectors = []
    safe_directions = []

    for direction, distance in sectors.items():
        if distance < safety_distance:
            obstacle_sectors.append(direction)
        else:
            safe_directions.append(direction)

    # Find closest obstacle
    closest_direction = min(sectors.keys(), key=lambda k: sectors[k])
    closest_distance = sectors[closest_direction]

    return {
        'obstacles_detected': len(obstacle_sectors) > 0,
        'obstacle_sectors': obstacle_sectors,
        'safe_directions': safe_directions,
        'closest_obstacle': {'direction': closest_direction, 'distance': closest_distance},
        'sectors': sectors
    }

def choose_avoidance_direction_enhanced(obstacle_info, target_angle, current_heading, last_action=None, consecutive_count=0, stored_target_direction=None):
    """
    Enhanced obstacle avoidance with target memory and multi-obstacle handling
    Rationale: Maintains target awareness and prevents oscillation with multiple obstacles

    Args:
        obstacle_info: result from detect_obstacles()
        target_angle: desired heading to target (radians)
        current_heading: current robot heading (radians)
        last_action: previous avoidance action to prevent oscillation
        consecutive_count: number of consecutive similar actions
        stored_target_direction: remembered target direction when obstacles first appeared

    Returns:
        dict with enhanced avoidance strategy:
        - 'action': 'turn_left', 'turn_right', 'go_straight', 'back_up', or 'stop'
        - 'reason': explanation of decision
        - 'urgency': 'low', 'medium', 'high', 'critical' based on obstacles
        - 'mode': 'navigating', 'avoiding', 'escaping'
        - 'target_awareness': True if considering target direction
        - 'using_stored_target': True if using stored target direction
    """
    if not obstacle_info or not obstacle_info['obstacles_detected']:
        return {
            'action': 'go_straight',
            'reason': 'No obstacles detected',
            'urgency': 'low',
            'mode': 'navigating',
            'target_awareness': True,
            'using_stored_target': False
        }

    sectors = obstacle_info['sectors']
    closest_dist = obstacle_info['closest_obstacle']['distance']
    obstacle_sectors = obstacle_info['obstacle_sectors']

    # Enhanced urgency levels
    if closest_dist < 0.6:
        urgency = 'critical'
    elif closest_dist < 0.8:
        urgency = 'high'
    elif closest_dist < 1.2:
        urgency = 'medium'
    else:
        urgency = 'low'

    # Analyze obstacle distribution
    front_blocked = 'front' in obstacle_sectors
    left_blocked = 'left' in obstacle_sectors or 'front_left' in obstacle_sectors
    right_blocked = 'right' in obstacle_sectors or 'front_right' in obstacle_sectors

    # Count total blocked sectors
    blocked_count = len(obstacle_sectors)

    # Calculate target direction preference - use stored target if available
    effective_target_angle = stored_target_direction if stored_target_direction is not None else target_angle
    using_stored_target = stored_target_direction is not None

    target_relative = angle_difference(current_heading, effective_target_angle)
    target_prefers_left = target_relative > 0.2  # Target significantly to left
    target_prefers_right = target_relative < -0.2  # Target significantly to right

    # CRITICAL SITUATION: Very close obstacles or surrounded
    if urgency == 'critical' or blocked_count >= 4:
        if closest_dist < 0.4:
            return {
                'action': 'back_up',
                'reason': f'Critical distance {closest_dist:.2f}m, backing up',
                'urgency': urgency,
                'mode': 'escaping',
                'target_awareness': False
            }

        # Emergency turn away from closest obstacle
        closest_dir = obstacle_info['closest_obstacle']['direction']
        if closest_dir in ['left', 'front_left']:
            action = 'turn_right'
            reason = f'Emergency: closest obstacle {closest_dir}'
        else:
            action = 'turn_left'
            reason = f'Emergency: closest obstacle {closest_dir}'

        return {
            'action': action,
            'reason': reason,
            'urgency': urgency,
            'mode': 'escaping',
            'target_awareness': False
        }

    # MULTIPLE OBSTACLES: Smart handling to prevent oscillation
    if blocked_count >= 2:
        # Prevent oscillation by checking last action
        if consecutive_count > 3 and last_action in ['turn_left', 'turn_right']:
            # Been turning same direction too long, try opposite or back up
            if last_action == 'turn_left' and not right_blocked:
                return {
                    'action': 'turn_right',
                    'reason': 'Breaking oscillation, switching direction',
                    'urgency': urgency,
                    'mode': 'avoiding',
                    'target_awareness': True
                }
            elif last_action == 'turn_right' and not left_blocked:
                return {
                    'action': 'turn_left',
                    'reason': 'Breaking oscillation, switching direction',
                    'urgency': urgency,
                    'mode': 'avoiding',
                    'target_awareness': True
                }
            else:
                return {
                    'action': 'back_up',
                    'reason': 'Breaking oscillation, backing up',
                    'urgency': urgency,
                    'mode': 'escaping',
                    'target_awareness': False
                }

        # Choose best available direction considering target
        if front_blocked:
            if not left_blocked and not right_blocked:
                # Both sides clear - choose based on target
                if target_prefers_left:
                    action, reason = 'turn_left', 'Multiple obstacles, target prefers left'
                elif target_prefers_right:
                    action, reason = 'turn_right', 'Multiple obstacles, target prefers right'
                else:
                    # Target straight ahead, choose side with more clearance
                    left_clearance = sectors.get('left', 0)
                    right_clearance = sectors.get('right', 0)
                    if left_clearance > right_clearance:
                        action, reason = 'turn_left', 'Multiple obstacles, left has more clearance'
                    else:
                        action, reason = 'turn_right', 'Multiple obstacles, right has more clearance'
            elif not left_blocked:
                action, reason = 'turn_left', 'Multiple obstacles, only left clear'
            elif not right_blocked:
                action, reason = 'turn_right', 'Multiple obstacles, only right clear'
            else:
                action, reason = 'back_up', 'Surrounded by obstacles'

            return {
                'action': action,
                'reason': reason,
                'urgency': urgency,
                'mode': 'avoiding',
                'target_awareness': True,
                'using_stored_target': using_stored_target
            }

    # SINGLE OBSTACLE: Strategic avoidance with target awareness
    if front_blocked:
        if target_prefers_left and not left_blocked:
            return {
                'action': 'turn_left',
                'reason': 'Front blocked, target direction left clear',
                'urgency': urgency,
                'mode': 'avoiding',
                'target_awareness': True,
                'using_stored_target': using_stored_target
            }
        elif target_prefers_right and not right_blocked:
            return {
                'action': 'turn_right',
                'reason': 'Front blocked, target direction right clear',
                'urgency': urgency,
                'mode': 'avoiding',
                'target_awareness': True,
                'using_stored_target': using_stored_target
            }
        elif not left_blocked and not right_blocked:
            # Both sides clear, choose based on clearance
            left_clearance = sectors.get('left', 0)
            right_clearance = sectors.get('right', 0)
            if left_clearance > right_clearance:
                action, reason = 'turn_left', 'Front blocked, left has more clearance'
            else:
                action, reason = 'turn_right', 'Front blocked, right has more clearance'
        elif not left_blocked:
            action, reason = 'turn_left', 'Front blocked, only left clear'
        elif not right_blocked:
            action, reason = 'turn_right', 'Front blocked, only right clear'
        else:
            action, reason = 'back_up', 'Front blocked, sides blocked'

        return {
            'action': action,
            'reason': reason,
            'urgency': urgency,
            'mode': 'avoiding',
            'target_awareness': True,
            'using_stored_target': using_stored_target
        }

    # PATH CLEAR: Continue toward target or resume navigation
    # Check if we need to turn back toward target after avoidance
    # Use stored target direction if available for better resumption
    effective_angle_to_target = angle_difference(current_heading, effective_target_angle)

    if abs(effective_angle_to_target) > 0.3:  # More than ~17 degrees off target
        if effective_angle_to_target > 0:
            return {
                'action': 'turn_left',
                'reason': 'Path clear, turning back toward target' + (' (using stored target)' if using_stored_target else ''),
                'urgency': urgency,
                'mode': 'navigating',
                'target_awareness': True,
                'using_stored_target': using_stored_target
            }
        else:
            return {
                'action': 'turn_right',
                'reason': 'Path clear, turning back toward target' + (' (using stored target)' if using_stored_target else ''),
                'urgency': urgency,
                'mode': 'navigating',
                'target_awareness': True,
                'using_stored_target': using_stored_target
            }

    # Already facing target, go straight
    return {
        'action': 'go_straight',
        'reason': 'Path clear, heading toward target' + (' (using stored target)' if using_stored_target else ''),
        'urgency': urgency,
        'mode': 'navigating',
        'target_awareness': True,
        'using_stored_target': using_stored_target
    }

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

def execute_avoidance_action(action_info, base_speed=1.0):
    """
    Execute enhanced obstacle avoidance action with appropriate motor commands
    Rationale: Reactive behavior with backing up capability for critical situations

    Args:
        action_info: result from choose_avoidance_direction_enhanced()
        base_speed: base movement speed

    Returns:
        True if action executed, False if stopped
    """
    action = action_info['action']
    urgency = action_info['urgency']

    # Adjust speed based on urgency
    if urgency == 'critical':
        turn_speed = base_speed * 2.0  # Very fast reaction for critical situations
        forward_speed = base_speed * 0.2  # Very slow forward when critical
        backup_speed = base_speed * 0.8  # Moderate backup speed
    elif urgency == 'high':
        turn_speed = base_speed * 1.5  # Faster reaction for urgent situations
        forward_speed = base_speed * 0.3  # Slower forward when danger close
        backup_speed = base_speed * 0.6
    elif urgency == 'medium':
        turn_speed = base_speed * 1.2
        forward_speed = base_speed * 0.6
        backup_speed = base_speed * 0.4
    else:
        turn_speed = base_speed
        forward_speed = base_speed
        backup_speed = base_speed * 0.3

    if action == 'turn_left':
        set_robot_speeds(-turn_speed, turn_speed)
        return True
    elif action == 'turn_right':
        set_robot_speeds(turn_speed, -turn_speed)
        return True
    elif action == 'go_straight':
        set_robot_speeds(forward_speed, forward_speed)
        return True
    elif action == 'back_up':
        set_robot_speeds(-backup_speed, -backup_speed)
        return True
    elif action == 'stop':
        stop_robot()
        return False

    return False

def debug_target_memory_system(current_pos, target_name, avoidance_state, last_clear_direction):
    """
    Debug function to understand target memory issues
    """
    if target_name:
        nav_info = navigate_to_target(current_pos, target_name)
        if nav_info:
            target_angle, distance, target_pos = nav_info
            print(f"DEBUG TARGET MEMORY:")
            print(f"  Target: {target_name} at {target_pos}")
            print(f"  Current pos: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
            print(f"  Distance: {distance:.2f}m, Angle: {target_angle:.2f} rad")
            print(f"  Avoidance state: {avoidance_state}")
            print(f"  Last clear direction: {last_clear_direction}")
            return target_angle, distance
    return None, None

def navigate_with_obstacle_avoidance_enhanced(current_pos, current_heading, target_name, lidar_readings,
                                          last_action=None, consecutive_count=0, avoidance_mode="navigating"):
    """
    Enhanced navigation with target memory and improved multi-obstacle handling
    Rationale: Maintains target awareness during avoidance and prevents oscillation

    Args:
        current_pos: (x, y) robot position
        current_heading: robot heading in radians
        target_name: target destination name
        lidar_readings: current LiDAR sensor data
        last_action: previous action to prevent oscillation
        consecutive_count: number of consecutive similar actions
        avoidance_mode: current avoidance state

    Returns:
        dict with enhanced navigation status and actions taken
    """
    # Get navigation information
    nav_info = navigate_to_target(current_pos, target_name)
    if not nav_info:
        return {'status': 'error', 'message': 'Invalid target'}

    target_angle, distance, target_pos = nav_info

    # Detect obstacles
    obstacle_info = detect_obstacles(lidar_readings)
    if not obstacle_info:
        return {'status': 'error', 'message': 'LiDAR data unavailable'}

    # Choose enhanced avoidance action with memory
    # Use the global stored target direction for better target memory
    global last_clear_direction_to_target
    avoidance_decision = choose_avoidance_direction_enhanced(
        obstacle_info, target_angle, current_heading, last_action, consecutive_count,
        stored_target_direction=last_clear_direction_to_target
    )

    # Execute the action
    action_executed = execute_avoidance_action(avoidance_decision)

    # Determine if we should be in avoidance mode or can resume navigation
    if obstacle_info['obstacles_detected']:
        if avoidance_decision['urgency'] in ['critical', 'high']:
            new_mode = 'avoiding'
        else:
            new_mode = 'navigating'
    else:
        new_mode = 'navigating'

    return {
        'status': 'navigating',
        'target_distance': distance,
        'target_angle': target_angle,
        'obstacles_detected': obstacle_info['obstacles_detected'],
        'action': avoidance_decision['action'],
        'reason': avoidance_decision['reason'],
        'urgency': avoidance_decision['urgency'],
        'mode': avoidance_decision['mode'],
        'target_awareness': avoidance_decision['target_awareness'],
        'using_stored_target': avoidance_decision.get('using_stored_target', False),
        'action_executed': action_executed,
        'avoidance_mode': new_mode,
        'obstacle_count': len(obstacle_info['obstacle_sectors'])
    }

# Enhanced navigation state machine with obstacle avoidance memory
current_target = None
navigation_state = "waiting"  # States: "waiting", "turning", "driving", "completed"
avoidance_state = "navigating"  # Sub-states: "navigating", "avoiding", "resuming"
step_counter = 0  # For debug output
last_clear_direction_to_target = None  # Remember target direction when obstacles appear
avoidance_start_time = 0  # Track how long we've been avoiding
consecutive_avoidance_actions = 0  # Prevent oscillation
last_avoidance_action = None  # Track last action for oscillation prevention

print("=== Week 2 Enhanced: Navigation with Smart Obstacle Avoidance ===")
print("Available commands: red, green, ducks, balls")
print("Features: Target-aware avoidance, multi-obstacle handling, oscillation prevention")
print("Improvements: Backing up capability, urgency levels, target memory")
print("Click in 3D view and type command + Enter")

while (robot.step(timestep) != -1):
    step_counter += 1

    # Debug LiDAR configuration on first step
    if step_counter == 1:
        try:
            lidar_readings = get_lidar_data()
            if lidar_readings:
                print(f"=== LiDAR Configuration Debug ===")
                print(f"Number of LiDAR points: {len(lidar_readings)}")
                print(f"LiDAR range: {min(lidar_readings):.2f}m to {max(lidar_readings):.2f}m")
                print(f"Sample readings - Front area (indices 80-120): {[f'{r:.2f}' for r in lidar_readings[80:120:10]]}")
                print(f"Sample readings - Left area (indices 0-40): {[f'{r:.2f}' for r in lidar_readings[0:40:10]]}")
                print(f"Sample readings - Right area (indices 160-200): {[f'{r:.2f}' for r in lidar_readings[160:200:10]]}")
                print("=====================================")
            else:
                print("WARNING: No LiDAR data available!")
        except Exception as e:
            print(f"LiDAR debug error: {e}")

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
        # Check if we've reached the target first
        try:
            goals_in_range = get_goals_in_range(*current_pos)
            if current_target in goals_in_range:
                print(f'Reached {current_target}!')
                stop_robot()
                navigation_state = "completed"
                current_target = None
                # Reset avoidance memory
                avoidance_state = "navigating"
                last_clear_direction_to_target = None
                consecutive_avoidance_actions = 0
            else:
                # Use enhanced obstacle avoidance navigation with memory
                lidar_readings = get_lidar_data()

                nav_result = navigate_with_obstacle_avoidance_enhanced(
                    current_pos, current_heading, current_target, lidar_readings,
                    last_avoidance_action, consecutive_avoidance_actions, avoidance_state
                )

                # Update memory and state tracking
                current_action = nav_result['action']
                if last_avoidance_action == current_action:
                    consecutive_avoidance_actions += 1
                else:
                    consecutive_avoidance_actions = 1

                last_avoidance_action = current_action
                avoidance_state = nav_result['avoidance_mode']

                # Store target direction when obstacles first appear
                if nav_result['obstacles_detected'] and last_clear_direction_to_target is None:
                    last_clear_direction_to_target = nav_result['target_angle']
                elif not nav_result['obstacles_detected']:
                    last_clear_direction_to_target = None

                # Enhanced status reporting (every 100 steps to avoid spam)
                if step_counter % 100 == 0:
                    print(f"Navigation: {nav_result['action']} - {nav_result['reason']}")
                    print(f"  Mode: {nav_result['mode']}, Avoidance State: {avoidance_state}")
                    if nav_result['obstacles_detected']:
                        print(f"  Obstacles: {nav_result['obstacle_count']}, Urgency: {nav_result['urgency']}")
                        print(f"  Target Awareness: {nav_result['target_awareness']}")
                        if nav_result.get('using_stored_target', False):
                            print(f"  Using Stored Target: YES (maintaining original direction)")
                        if consecutive_avoidance_actions > 1:
                            print(f"  Consecutive actions: {consecutive_avoidance_actions}")
                    print(f"  Distance to {current_target}: {nav_result['target_distance']:.2f}m")

                    # Debug target memory system
                    debug_target_memory_system(current_pos, current_target, avoidance_state, last_clear_direction_to_target)

        except Exception as e:
            print(f"Navigation error: {e}")
            # Fallback: check distance manually and use simple navigation
            nav_info = navigate_to_target(current_pos, current_target)
            if nav_info and nav_info[1] < 0.8:  # Within 0.8m threshold
                print(f'Reached {current_target} (fallback detection)!')
                stop_robot()
                navigation_state = "completed"
                current_target = None
                # Reset avoidance memory
                avoidance_state = "navigating"
                last_clear_direction_to_target = None
                consecutive_avoidance_actions = 0
            else:
                # Simple fallback - just drive forward slowly
                print("Using fallback navigation")
                drive_forward(0.8)

    elif navigation_state == "completed":
        print("Mission completed! Waiting for next command...")
        navigation_state = "waiting"

    # Debug output every 5 seconds
    if step_counter % 500 == 0:
        print(f"Status: {navigation_state}, Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f}), Target: {current_target}")

        # Show obstacle detection status
        if navigation_state == "driving":
            try:
                lidar_readings = get_lidar_data()
                obstacle_info = detect_obstacles(lidar_readings)
                if obstacle_info and obstacle_info['obstacles_detected']:
                    closest = obstacle_info['closest_obstacle']
                    print(f"  Obstacles: {obstacle_info['obstacle_sectors']}")
                    print(f"  Closest: {closest['direction']} at {closest['distance']:.2f}m")
                else:
                    print("  No obstacles detected")
            except Exception as e:
                print(f"  Obstacle detection error: {e}")
    


