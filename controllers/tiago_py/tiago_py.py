#
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
print("Supported targets: 'balls', 'green', 'ducks', 'red'")
print("Type 'balls', 'green', 'ducks', or 'red' in 3D view and press enter to start navigation")
print("Expected output: 'Goal reached! Total distance covered: X.XX meters'")
print("=" * 50)

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

        # Command validation and parsing with comprehensive goal support
        # Algorithmic Decision: Case-insensitive matching with whitespace trimming
        # ensures robust command recognition across different input methods
        command_lower = command.strip().lower()

        # Multi-goal validation: Support for all 4 target types
        # Design Rationale: Extensible goal set enables future target additions
        # without architectural changes to the command processing system
        if command_lower in ['balls', 'green', 'ducks', 'red']:

            # State-based navigation control with collision prevention
            # Safety Mechanism: Prevents concurrent navigation attempts that could
            # cause system instability or conflicting robot behaviors
            if nav_controller.state in ["IDLE", "GOAL_REACHED"]:

                # Initiate PDDL-based navigation planning and execution
                # Integration Point: Bridges user commands with automated planning
                success = nav_controller.start_navigation_to_target(command_lower)
                if not success:
                    # Error handling: Graceful degradation with user feedback
                    print(f"Failed to start navigation to {command_lower} target")
            else:
                # Concurrent operation prevention with state reporting
                # User Experience: Clear feedback about system state prevents confusion
                print(f"Navigation already in progress (state: {nav_controller.state})")
        else:
            # Input validation with comprehensive error messaging
            # Design Principle: Clear error messages improve system usability
            print(f"Invalid command: '{command}'. Supported targets: 'balls', 'green', 'ducks', 'red'")

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

        # Conditional reporting: Only display active navigation states
        # Performance Optimization: Reduces log verbosity during idle periods
        if status['state'] != "IDLE":
            # Comprehensive status display: State, target, and progress metrics
            # Monitoring Integration: Provides data for performance analysis
            print(f"Status: {status['state']}, Target: {status['current_target']}, "
                  f"Distance covered: {status['distance_covered']:.2f}m")
    


