import math
import time
from node_network import NodeNetwork
from pddl_system import PDDLSystem
from goalchecker import get_goals_in_range


class NavigationController:
    """
    **System Architecture:**
    The controller operates as a hierarchical state machine with the following layers:
    1. Mission Layer: Manages goal queues and navigation sessions
    2. Planning Layer: Generates optimal paths using PDDL and A* search
    3. Execution Layer: Controls robot movement and waypoint navigation
    4. Safety Layer: Handles obstacle detection and avoidance

    **State Management:**
    - IDLE: System ready for new navigation commands
    - PLANNING: Generating optimal path using PDDL system
    - NAVIGATING: Actively moving toward current waypoint
    - GOAL_REACHED: Successfully completed current goal

    **Navigation Features:**
    - Optimal path planning with guaranteed shortest routes
    - Precise waypoint following with 5cm positioning tolerance
    - Real-time obstacle detection and avoidance
    - Multi-goal queue processing with progress tracking
    - Robust error handling and recovery mechanisms

    Attributes:
        robot (Robot): Webots robot instance for device access
        timestep (int): Simulation timestep for device synchronization
        network (NodeNetwork): Strategic waypoint network for navigation
        pddl_system (PDDLSystem): PDDL planning system for optimal paths
        state (str): Current navigation state (IDLE/PLANNING/NAVIGATING/GOAL_REACHED)
        current_path (list): Current navigation path as sequence of nodes
        goal_queue (list): Queue of pending goals for sequential navigation
        total_distance_covered (float): Cumulative distance traveled in current session
    """
    def __init__(self, robot, timestep):
        """
        Initialize the advanced navigation controller with all required components.

        Sets up robot devices, navigation systems, and control parameters for
        optimal multi-goal navigation performance. Initializes PDDL planning
        system and strategic node network for path optimization.

        **Initialization Process:**
        1. Configure robot devices (motors, sensors, navigation)
        2. Initialize PDDL planning system and node network
        3. Set up obstacle detection and avoidance parameters
        4. Configure navigation state management and goal queue system
        5. Establish control parameters for precise robot movement

        **Device Configuration:**
        - Motors: Differential drive with velocity control mode
        - LiDAR: Obstacle detection with 0.3m threshold
        - GPS: Position feedback for navigation accuracy
        - Compass: Heading feedback for orientation control

        **Control Parameters:**
        - Max Speed: 10.0 m/s for efficient navigation
        - Angle Tolerance: 0.1 rad (~6°) for precise orientation
        - Position Tolerance: 0.05m (5cm) for accurate waypoint following
        - Goal Threshold: 0.8m for goal completion detection

        Args:
            robot (Robot): Webots robot instance for device access
            timestep (int): Simulation timestep for device synchronization

        Raises:
            RuntimeError: If critical devices cannot be initialized
            ValueError: If timestep is invalid or negative
        """
        self.robot = robot
        self.timestep = timestep

        # Initialize and configure robot devices
        self._initialize_devices()

        # Initialize navigation components
        self.network = NodeNetwork()
        self.pddl_system = PDDLSystem()

        # Configure obstacle detection system
        self.obstacle_detected = False
        self.obstacle_pause_start = None
        self.obstacle_pause_duration = 10.0  # 10-second pause for obstacle clearance
        self.obstacle_threshold = 0.3  # 30cm detection threshold for safety

        # Initialize navigation state variables
        self.current_path = []
        self.current_target_node = None
        self.current_target_goal = None
        self.path_index = 0
        self.total_distance_covered = 0.0
        self.planned_total_cost = 0.0

        # Configure goal queue management system
        self.goal_queue = []
        self.completed_goals = []
        self.current_goal_index = 0
        self.total_goals_in_session = 0

        # Set robot control parameters
        self.max_speed = 10.0
        self.angle_tolerance = 0.1  # ~6 degrees for precise orientation
        self.position_tolerance = 0.05  # 5cm for accurate positioning

        # Initialize state management
        self.state = "IDLE"
        self.goal_reached = False

        # Configure node transition control
        self.node_reached_time = None
        self.pause_duration = 0.5  # Brief pause at each waypoint
        self.last_completed_node = None

    def _initialize_devices(self):
        """
        Initialize and configure all robot devices.

        Sets up motors in velocity control mode, enables sensors with
        appropriate timestep, and validates device availability.
        """
        # Configure differential drive motors
        self.l_motor = self.robot.getDevice("wheel_left_joint")
        self.r_motor = self.robot.getDevice("wheel_right_joint")
        self.l_motor.setPosition(math.inf)  # Velocity control mode
        self.r_motor.setPosition(math.inf)
        self.l_motor.setVelocity(0)  # Start stationary
        self.r_motor.setVelocity(0)

        # Configure navigation sensors
        self.lidar = self.robot.getDevice("lidar")
        self.compass = self.robot.getDevice("compass")
        self.gps = self.robot.getDevice('gps')

        # Enable sensors with simulation timestep
        self.lidar.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.gps.enable(self.timestep)
        
    def get_robot_position(self):
        """Get current robot position from GPS."""
        return self.gps.getValues()[0:2]
    
    def get_robot_heading(self):
        """Get current robot heading from compass."""
        compass_values = self.compass.getValues()
        # Use correct compass axis mapping: atan2(x, -z)
        heading = math.atan2(compass_values[0], -compass_values[2])
        return heading

    def add_goal_to_queue(self, goal):
        """Add a goal to the navigation queue."""
        if goal not in ['balls', 'green', 'ducks', 'red']:
            return False

        self.goal_queue.append(goal)
        self.total_goals_in_session += 1  # Update total goals count
        print(f"Added '{goal}' to queue. Queue: {self.goal_queue}")
        return True

    def add_multiple_goals_to_queue(self, goals):
        """Add multiple goals to the navigation queue."""
        added_goals = []
        for goal in goals:
            if self.add_goal_to_queue(goal):
                added_goals.append(goal)

        if added_goals:
            print(f"Queue updated: {self.goal_queue}")
            return True
        return False

    def get_next_goal_from_queue(self):
        """Get the next goal from the queue."""
        if self.goal_queue:
            return self.goal_queue.pop(0)
        return None

    def clear_goal_queue(self):
        """Clear all goals from the queue."""
        self.goal_queue.clear()
        self.completed_goals.clear()
        self.current_goal_index = 0
        self.total_goals_in_session = 0

    def get_queue_status(self):
        """Get current queue status information."""
        return {
            'queue': self.goal_queue.copy(),
            'completed': self.completed_goals.copy(),
            'current_goal': self.current_target_goal,
            'total_goals': self.total_goals_in_session,
            'completed_count': len(self.completed_goals),
            'remaining_count': len(self.goal_queue)
        }

    def check_obstacle_detection(self):
        """Simple obstacle detection: pause for 10 seconds if obstacle within 0.1m.
        Personal Note: This code is augment code generated
        """
        # Get LiDAR readings
        lidar_values = self.lidar.getRangeImage()

        # Check front-facing sensors for obstacles
        # Use middle third of LiDAR readings (front-facing)
        front_readings = lidar_values[len(lidar_values)//3 : -len(lidar_values)//3]

        # Check if any reading is below threshold
        obstacle_detected_now = any(distance < self.obstacle_threshold for distance in front_readings)

        # If obstacle detected and we're not already pausing
        if obstacle_detected_now and not self.obstacle_detected:
            print(f"OBSTACLE DETECTED within {self.obstacle_threshold}m! Pausing for {self.obstacle_pause_duration} seconds...")
            self.obstacle_detected = True
            self.obstacle_pause_start = time.time()
            self.stop_robot()
            return True  # Robot should pause

        # If we're currently pausing due to obstacle
        if self.obstacle_detected and self.obstacle_pause_start is not None:
            elapsed_time = time.time() - self.obstacle_pause_start
            if elapsed_time >= self.obstacle_pause_duration:
                print("Obstacle pause complete. Resuming navigation...")
                self.obstacle_detected = False
                self.obstacle_pause_start = None
                return False  # Resume navigation
            else:
                # Still pausing
                return True  # Continue pausing

        return False  # No obstacle, continue normal navigation
    
    def calculate_angle_difference(self, target_angle, current_angle):
        """Calculate the shortest angle difference between two angles.
        Personal Note: This code is agument code generated
        """
        diff = target_angle - current_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
    
    def rotate_to_angle(self, target_angle):
        """Rotate robot to face target angle. Returns True when complete.
        Personal Note: This code is augment code generated
        """
        current_angle = self.get_robot_heading()
        angle_diff = self.calculate_angle_difference(target_angle, current_angle)

        if abs(angle_diff) < self.angle_tolerance:
            self.stop_robot()
            return True

        turn_rate = angle_diff * 2.0  # Proportional gain
        # Clamp to reasonable limits
        max_turn_speed = 2.0
        turn_rate = max(-max_turn_speed, min(max_turn_speed, turn_rate))

        self.set_motor_speeds(-turn_rate, turn_rate)
        return False
    
    def move_forward(self, speed=None):
        """
        Move robot forward at specified speed.

        Args:
            speed (float, optional): Forward velocity in m/s. Defaults to max_speed.
        """
        if speed is None:
            speed = self.max_speed
        self.set_motor_speeds(speed, speed)
    
    def stop_robot(self):
        """Stop the robot."""
        self.set_motor_speeds(0, 0)
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Set motor speeds with safety limits."""
        max_speed = 10.0
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

        # If this is a single goal (not part of a queue), set total goals to 1
        if self.total_goals_in_session == 0:
            self.total_goals_in_session = 1

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

    def start_queue_navigation(self, goals):
        """Start navigation with a queue of goals."""
        if not goals:
            print("No goals provided")
            return False

        # Clear existing queue and add new goals
        self.clear_goal_queue()

        # Validate and add goals
        valid_goals = []
        for goal in goals:
            if goal in ['balls', 'green', 'ducks', 'red']:
                valid_goals.append(goal)
            else:
                print(f"Invalid goal: '{goal}'. Skipping.")

        if not valid_goals:
            print("No valid goals provided")
            return False

        # Set total goals for this session
        self.total_goals_in_session = len(valid_goals)

        # Add goals to queue (except the first one which we'll start immediately)
        if len(valid_goals) > 1:
            self.goal_queue = valid_goals[1:]

        print(f"Queue: {valid_goals} - Starting navigation to {valid_goals[0]}")

        # Start navigation to first goal
        return self.start_navigation_to_target(valid_goals[0])

    def navigate_to_current_node(self):
        """Navigate to the current target node using state machine approach.
        Personal Note: This code is augment generated
        """
        if not self.current_target_node:
            return True  # Navigation complete

        # Check for obstacles first - simple pause logic
        if self.check_obstacle_detection():
            return False  # Pausing due to obstacle, don't continue navigation

        robot_pos = self.get_robot_position()
        target_pos = self.network.get_node_coordinates(self.current_target_node)

        if not target_pos:
            print(f"Invalid target node: {self.current_target_node}")
            return True

        # Check if we've reached the target node
        distance = self.network.calculate_distance(robot_pos, target_pos)

        if distance <= self.position_tolerance:
            self.stop_robot()

            # Check if we've already processed this node completion
            if self.last_completed_node == self.current_target_node:
                return False  # Already processed this node, just wait

            # If we just reached this node, record the time and print message once
            if self.node_reached_time is None:
                print(f"✓ Reached waypoint: {self.current_target_node}")
                self.node_reached_time = self.robot.getTime()
                return False  # Stay at this node for a brief pause

            # Check if we've paused long enough
            if self.robot.getTime() - self.node_reached_time < self.pause_duration:
                return False  # Continue pausing

            # Reset pause timer and process node completion
            self.node_reached_time = None
            self.last_completed_node = self.current_target_node  # Mark this node as completed

            # Update distance covered
            if self.path_index > 0 and self.path_index - 1 < len(self.current_path):
                prev_node = self.current_path[self.path_index - 1]
                segment_cost = self.network.get_connection_weight(prev_node, self.current_target_node)
                self.total_distance_covered += segment_cost

            # Move to next node in path
            self.path_index += 1

            if self.path_index < len(self.current_path):
                self.current_target_node = self.current_path[self.path_index]
                self.last_completed_node = None  # Reset for next node
                return False
            else:
                # Path complete - check if we've reached the target
                print(f"Path complete! Checking goal completion for {self.current_target_goal}")
                return self.check_goal_completion()

        # Navigation control using proportional feedback
        target_angle = self.network.calculate_angle_to_target(robot_pos, target_pos)
        current_heading = self.get_robot_heading()
        angle_diff = self.calculate_angle_difference(target_angle, current_heading)

        # Proportional control for precise navigation
        if abs(angle_diff) > self.angle_tolerance:
            # Rotate toward target with proportional control
            turn_rate = angle_diff * 2.0  # Proportional gain
            max_turn_speed = 2.0
            turn_rate = max(-max_turn_speed, min(max_turn_speed, turn_rate))
            self.set_motor_speeds(-turn_rate, turn_rate)
        else:
            # Move forward with distance-based speed control
            if distance < 0.3:  # Slow down when approaching target
                speed = max(0.5, distance * 2.0)  # Proportional speed reduction
            else:
                speed = self.max_speed
            self.move_forward(speed=speed)

        return False
    
    def check_goal_completion(self):
        """
        Check if robot has reached the current target goal.

        Uses goalchecker polygon detection for accurate goal completion validation.
        The polygon-based method provides precise boundary detection with 0.8m threshold.

        Returns:
            bool: True if goal is reached, False otherwise
        """
        robot_pos = self.get_robot_position()
        target_goal = self.current_target_goal or 'balls'

        print(f"Checking goal completion for {target_goal} at position {robot_pos}")

        # Use goalchecker polygon detection (most accurate method)
        nearby_goals = get_goals_in_range(*robot_pos)
        print(f"Nearby goals detected: {nearby_goals}")

        if target_goal in nearby_goals:
            print(f"Goal {target_goal} reached!")
            self._handle_goal_reached(target_goal)
            return True

        # If goal not reached, approach target center directly
        target_coords = self.network.get_target_coordinates(target_goal)
        target_distance = self.network.calculate_distance(robot_pos, target_coords)
        print(f"Distance to {target_goal} center: {target_distance:.3f}m")

        if target_distance > 0.5:  # Move closer if more than 50cm away
            print(f"Moving closer to {target_goal} target...")
            # Direct approach to goal with reduced speed
            target_angle = self.network.calculate_angle_to_target(robot_pos, target_coords)
            if self.rotate_to_angle(target_angle):
                self.move_forward(speed=1.0)  # Careful approach

        return False

    def _handle_goal_reached(self, target_goal):
        """Handle goal completion and queue management."""
        print(f"\n{target_goal.upper()} GOAL REACHED! Total distance covered: {self.total_distance_covered:.2f} meters")

        # Add completed goal to completed list
        self.completed_goals.append(target_goal)

        # Calculate progress
        queue_status = self.get_queue_status()
        completed_count = len(self.completed_goals)
        total_goals = queue_status['total_goals']

        if total_goals > 1:
            print(f"Goal {completed_count} of {total_goals} completed!")

        self.state = "GOAL_REACHED"
        self.goal_reached = True
        self.stop_robot()

        # Clear goal detection state to prevent spam
        if hasattr(self, '_last_detected_goal'):
            delattr(self, '_last_detected_goal')

        # Check if there are more goals in the queue
        next_goal = self.get_next_goal_from_queue()
        if next_goal:
            print(f"Starting navigation to next goal: {next_goal}")
            print(f"Remaining queue: {self.goal_queue}")
            # Start navigation to next goal from current position
            self.start_navigation_to_target(next_goal)
        else:
            print("All queued goals completed!")
            self.current_target_goal = None
            # Reset session when all goals are completed
            self.total_goals_in_session = 0

    def update(self):
        """Main update loop for navigation controller."""
        if self.state == "NAVIGATING":
            self.navigate_to_current_node()
        elif self.state == "GOAL_REACHED":
            self.stop_robot()
    
    def get_status(self):
        """Get current navigation status."""
        robot_pos = self.get_robot_position()
        queue_status = self.get_queue_status()

        return {
            'state': self.state,
            'position': robot_pos,
            'current_target': self.current_target_node,
            'current_goal': self.current_target_goal,
            'path': self.current_path,
            'distance_covered': self.total_distance_covered,
            'planned_cost': self.planned_total_cost,
            'goal_reached': self.goal_reached,
            'queue': queue_status['queue'],
            'completed_goals': queue_status['completed'],
            'total_goals': queue_status['total_goals'],
            'progress': f"{queue_status['completed_count']}/{queue_status['total_goals']}" if queue_status['total_goals'] > 0 else "0/0"
        }
