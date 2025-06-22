"""
Navigation Controller for TiaGo Robot

This module implements the main navigation logic with PDDL planning,
robust movement control, and path cost tracking for the balls target.
"""

import math
import time
from node_network import NodeNetwork
from pddl_system import PDDLSystem
from goalchecker import get_goals_in_range


class NavigationController:
    def __init__(self, robot, timestep):
        """Initialize the navigation controller."""
        self.robot = robot
        self.timestep = timestep
        
        # Initialize devices
        self.l_motor = robot.getDevice("wheel_left_joint")
        self.r_motor = robot.getDevice("wheel_right_joint")
        self.lidar = robot.getDevice("lidar")
        self.compass = robot.getDevice("compass")
        self.gps = robot.getDevice('gps')
        
        # Enable devices
        self.lidar.enable(timestep)
        self.compass.enable(timestep)
        self.gps.enable(timestep)
        
        # Set motor modes
        self.l_motor.setPosition(math.inf)
        self.r_motor.setPosition(math.inf)
        self.l_motor.setVelocity(0)
        self.r_motor.setVelocity(0)
        
        # Initialize navigation components
        self.network = NodeNetwork()
        self.pddl_system = PDDLSystem()
        
        # Navigation state
        self.current_path = []
        self.current_target_node = None
        self.current_target_goal = None  # Which goal we're navigating to ('balls' or 'green')
        self.path_index = 0
        self.total_distance_covered = 0.0
        self.planned_total_cost = 0.0

        # Robot parameters
        self.max_speed = 3.0
        self.angle_tolerance = 0.1  # ~6 degrees tolerance like your working code
        self.position_tolerance = 0.05  # Very tight tolerance - 5cm for precise positioning

        # State tracking
        self.state = "IDLE"  # IDLE, PLANNING, NAVIGATING, GOAL_REACHED
        self.goal_reached = False

        # Node transition tracking
        self.node_reached_time = None
        self.pause_duration = 0.5  # Pause for 0.5 seconds when reaching a node
        self.last_completed_node = None  # Track which node we last completed to prevent duplicate processing
        self.last_completed_node = None  # Track which node we last completed
        
    def get_robot_position(self):
        """Get current robot position from GPS."""
        return self.gps.getValues()[0:2]
    
    def get_robot_heading(self):
        """Get current robot heading from compass."""
        compass_values = self.compass.getValues()
        # Use correct compass axis mapping: atan2(x, -z)
        heading = math.atan2(compass_values[0], -compass_values[2])
        # Debug output to verify compass readings
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0

        if self._debug_counter % 20 == 0:  # Print every 20 calls to avoid spam
            print(f"DEBUG: Compass values: [{compass_values[0]:.3f}, {compass_values[1]:.3f}, {compass_values[2]:.3f}]")
            print(f"DEBUG: Calculated heading: {heading:.3f} rad ({heading*180/3.14159:.1f}°)")

        return heading
    
    def calculate_angle_difference(self, target_angle, current_angle):
        """Calculate the shortest angle difference between two angles."""
        diff = target_angle - current_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
    
    def rotate_to_angle(self, target_angle):
        """Rotate robot to face target angle. Returns True when complete."""
        current_angle = self.get_robot_heading()
        angle_diff = self.calculate_angle_difference(target_angle, current_angle)

        if abs(angle_diff) < self.angle_tolerance:
            self.stop_robot()
            return True

        # Use proportional control like your working code
        turn_rate = angle_diff * 2.0  # Proportional gain
        # Clamp to reasonable limits
        max_turn_speed = 2.0
        turn_rate = max(-max_turn_speed, min(max_turn_speed, turn_rate))

        self.set_motor_speeds(-turn_rate, turn_rate)
        return False
    
    def move_forward(self, speed=None):
        """Move robot forward at specified speed."""
        if speed is None:
            speed = self.max_speed
        print(f"Moving forward at speed: {speed}")
        self.set_motor_speeds(speed, speed)
    
    def stop_robot(self):
        """Stop the robot."""
        self.set_motor_speeds(0, 0)
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds with safety limits."""
        # Clamp speeds to safe limits like your working code
        max_speed = 3.0
        left_speed = max(-max_speed, min(max_speed, left_speed))
        right_speed = max(-max_speed, min(max_speed, right_speed))

        self.l_motor.setVelocity(left_speed)
        self.r_motor.setVelocity(right_speed)
    
    def start_navigation_to_target(self, target='balls'):
        """Start navigation to specified target using PDDL planning."""
        if self.state not in ["IDLE", "GOAL_REACHED"]:
            print("Navigation already in progress")
            return False

        # Reset state for new navigation
        if self.state == "GOAL_REACHED":
            print(f"Starting new navigation from current position to {target}")
            self.goal_reached = False

        print(f"\n=== Starting Navigation to {target.title()} Target ===")
        self.state = "PLANNING"
        self.current_target_goal = target  # Store which goal we're navigating to

        # Reset navigation state variables
        self.current_path = []
        self.current_target_node = None
        self.path_index = 0
        self.total_distance_covered = 0.0
        self.planned_total_cost = 0.0
        self.node_reached_time = None
        self.last_completed_node = None

        robot_pos = self.get_robot_position()
        print(f"Robot position: {robot_pos}")

        # Generate navigation plan using PDDL
        path, total_cost = self.pddl_system.plan_navigation(robot_pos, target)

        if path and len(path) > 1:
            self.current_path = path
            self.path_index = 1  # Start with second node (first is current position)
            self.current_target_node = path[1]
            self.planned_total_cost = total_cost
            self.total_distance_covered = 0.0
            self.state = "NAVIGATING"

            print(f"Navigation plan: {path}")
            print(f"Planned total cost: {total_cost:.3f}m")
            print(f"First target: {self.current_target_node}")
            return True
        else:
            print("Failed to generate navigation plan")
            self.state = "IDLE"
            return False

    def start_navigation_to_balls(self):
        """Start navigation to balls target (backward compatibility)."""
        return self.start_navigation_to_target('balls')
    
    def navigate_to_current_node(self):
        """Navigate to the current target node using state machine approach."""
        if not self.current_target_node:
            return True  # Navigation complete

        robot_pos = self.get_robot_position()
        target_pos = self.network.get_node_coordinates(self.current_target_node)

        if not target_pos:
            print(f"Invalid target node: {self.current_target_node}")
            return True

        # Check if we've reached the target node
        distance = self.network.calculate_distance(robot_pos, target_pos)

        print(f"Navigating to {self.current_target_node}: robot={robot_pos}, target={target_pos}, distance={distance:.3f}m")

        if distance <= self.position_tolerance:
            print(f"✓ Reached node: {self.current_target_node} (distance: {distance:.3f}m)")
            self.stop_robot()

            # Check if we've already processed this node completion
            if self.last_completed_node == self.current_target_node:
                return False  # Already processed this node, just wait

            # If we just reached this node, record the time
            if self.node_reached_time is None:
                self.node_reached_time = self.robot.getTime()
                print(f"DEBUG: Starting pause at node {self.current_target_node}")
                return False  # Stay at this node for a brief pause

            # Check if we've paused long enough
            if self.robot.getTime() - self.node_reached_time < self.pause_duration:
                return False  # Continue pausing

            # Reset pause timer and process node completion
            self.node_reached_time = None
            self.last_completed_node = self.current_target_node  # Mark this node as completed
            print(f"DEBUG: Path info - current_path: {self.current_path}, path_index: {self.path_index}, len: {len(self.current_path)}")

            # Update distance covered
            if self.path_index > 0 and self.path_index - 1 < len(self.current_path):
                prev_node = self.current_path[self.path_index - 1]
                segment_cost = self.network.get_connection_weight(prev_node, self.current_target_node)
                self.total_distance_covered += segment_cost
                print(f"Segment cost: {segment_cost:.3f}m, Total covered: {self.total_distance_covered:.3f}m")

            # Move to next node in path (only increment once per node)
            self.path_index += 1
            print(f"DEBUG: Incremented path_index to {self.path_index}")

            if self.path_index < len(self.current_path):
                self.current_target_node = self.current_path[self.path_index]
                self.last_completed_node = None  # Reset for next node
                print(f"Next target: {self.current_target_node}")
                return False
            else:
                # Path complete - check if we've reached the target
                print(f"DEBUG: Path complete, checking goal completion for {self.current_target_goal}")
                return self.check_goal_completion()

        # Simplified navigation logic based on your working code
        target_angle = self.network.calculate_angle_to_target(robot_pos, target_pos)
        current_heading = self.get_robot_heading()
        angle_diff = self.calculate_angle_difference(target_angle, current_heading)

        print(f"Target angle: {target_angle:.3f} rad ({target_angle*180/3.14159:.1f}°)")
        print(f"Current heading: {current_heading:.3f} rad ({current_heading*180/3.14159:.1f}°)")
        print(f"Angle difference: {angle_diff:.3f} rad ({angle_diff*180/3.14159:.1f}°)")

        # Simple approach like your working code with distance-based speed control
        if abs(angle_diff) > self.angle_tolerance:
            print("↻ Need to rotate")
            # Use proportional control for turning
            turn_rate = angle_diff * 2.0  # Proportional gain
            max_turn_speed = 2.0
            turn_rate = max(-max_turn_speed, min(max_turn_speed, turn_rate))
            self.set_motor_speeds(-turn_rate, turn_rate)
        else:
            print("→ Moving forward")
            # Slow down as we approach the target for precise positioning
            if distance < 0.3:  # Within 30cm of target
                speed = max(0.5, distance * 2.0)  # Proportional speed reduction
                print(f"  Approaching target - reduced speed: {speed:.2f}")
            else:
                speed = 1.5
            self.move_forward(speed=speed)

        return False
    
    def check_goal_completion(self):
        """Check if robot has reached the current target goal."""
        robot_pos = self.get_robot_position()
        target_goal = self.current_target_goal or 'balls'  # Default to balls for backward compatibility

        # Check using goalchecker module
        nearby_goals = get_goals_in_range(*robot_pos)
        if target_goal in nearby_goals:
            print(f"\n🎯 {target_goal.upper()} GOAL REACHED! Total distance covered: {self.total_distance_covered:.2f} meters")
            self.state = "GOAL_REACHED"
            self.goal_reached = True
            self.stop_robot()
            return True

        # Also check using network distance calculation
        if self.network.is_goal_reached(robot_pos, target_goal):
            print(f"\n🎯 {target_goal.upper()} GOAL REACHED! Total distance covered: {self.total_distance_covered:.2f} meters")
            self.state = "GOAL_REACHED"
            self.goal_reached = True
            self.stop_robot()
            return True

        # If we completed the path but didn't reach goal, try to get closer
        target_coords = self.network.get_target_coordinates(target_goal)
        target_distance = self.network.calculate_distance(robot_pos, target_coords)
        print(f"Path complete. Distance to {target_goal}: {target_distance:.3f}m")

        if target_distance > self.network.goal_threshold:
            print(f"Moving directly toward {target_goal} target...")
            target_angle = self.network.calculate_angle_to_target(robot_pos, target_coords)
            if self.rotate_to_angle(target_angle):
                self.move_forward(speed=1.0)  # Slower approach

        return False
    
    def update(self):
        """Main update loop for navigation controller."""
        if self.state == "IDLE":
            # Waiting for navigation command
            pass
        elif self.state == "PLANNING":
            # Planning is handled in start_navigation_to_balls
            pass
        elif self.state == "NAVIGATING":
            self.navigate_to_current_node()
        elif self.state == "GOAL_REACHED":
            # Goal reached, stop
            self.stop_robot()
    
    def get_status(self):
        """Get current navigation status."""
        robot_pos = self.get_robot_position()
        return {
            'state': self.state,
            'position': robot_pos,
            'current_target': self.current_target_node,
            'path': self.current_path,
            'distance_covered': self.total_distance_covered,
            'planned_cost': self.planned_total_cost,
            'goal_reached': self.goal_reached
        }
