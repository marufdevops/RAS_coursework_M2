"""
Behavior Coordination System

This module implements a subsumption architecture to coordinate global planning,
local navigation, and obstacle avoidance behaviors for the TiaGo robot.

Author: Mission 2 Navigation System
"""

import math
import time
from typing import Tuple, Optional, Dict, List
from enum import Enum

from global_planner import GlobalPathPlanner
from sensor_processor import SensorProcessor
from obstacle_avoidance import ObstacleAvoidance, AvoidanceState


class NavigationMode(Enum):
    """Navigation modes for behavior coordination."""
    IDLE = "idle"
    GLOBAL_PLANNING = "global_planning"
    WAYPOINT_NAVIGATION = "waypoint_navigation"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    TARGET_APPROACH = "target_approach"
    GOAL_REACHED = "goal_reached"


class BehaviorCoordinator:
    """
    Coordinates multiple navigation behaviors using subsumption architecture.
    
    Manages the interaction between global path planning, local navigation,
    and reactive obstacle avoidance to achieve robust navigation behavior.
    """
    
    def __init__(self):
        """Initialize the behavior coordination system."""
        # Component initialization
        self.global_planner = GlobalPathPlanner()
        self.sensor_processor = SensorProcessor()
        self.obstacle_avoidance = ObstacleAvoidance()
        
        # Navigation state
        self.mode = NavigationMode.IDLE
        self.current_target = None
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.goal_position = None
        
        # Control parameters
        self.waypoint_tolerance = 1.0  # Distance to consider waypoint reached
        self.goal_tolerance = 0.8  # Distance to consider goal reached
        self.max_linear_velocity = 2.0
        self.max_angular_velocity = 1.0
        
        # Navigation control
        self.kp_linear = 1.5  # Proportional gain for linear velocity
        self.kp_angular = 2.0  # Proportional gain for angular velocity
        
        # State tracking
        self.last_position = None
        self.last_heading = None
        self.navigation_start_time = None
        
        # Performance monitoring
        self.stuck_detection_enabled = True
        self.max_navigation_time = 60.0  # Maximum time for single navigation task
        
    def set_navigation_target(self, target: str) -> bool:
        """
        Set a new navigation target and initiate planning.
        
        Args:
            target: Target name (red, green, ducks, balls)
            
        Returns:
            True if target is valid and planning succeeded, False otherwise
        """
        print(f"Setting navigation target: {target}")
        
        # Validate target
        if target not in ['red', 'green', 'ducks', 'balls']:
            print(f"Invalid target: {target}")
            return False
        
        self.current_target = target
        self.navigation_start_time = time.time()
        
        # Get current position for planning
        current_pos = "start-pos"  # Default starting position
        if self.last_position is not None:
            current_pos = self.global_planner.get_nearest_waypoint(
                self.last_position[0], self.last_position[1]
            )
        
        # Plan global path
        waypoints = self.global_planner.plan_path(target, current_pos)
        if waypoints is None:
            print(f"Failed to plan path to {target}")
            self.mode = NavigationMode.IDLE
            return False
        
        self.current_waypoints = waypoints
        self.current_waypoint_index = 0
        self.goal_position = self.global_planner.get_coordinates(
            self.global_planner.target_locations[target]
        )
        
        print(f"Planned path: {waypoints}")
        self.mode = NavigationMode.WAYPOINT_NAVIGATION
        return True
    
    def update(self, gps_values: List[float], compass_values: List[float], 
               lidar_values: List[float]) -> Tuple[float, float]:
        """
        Main update loop for behavior coordination.
        
        Args:
            gps_values: GPS sensor data
            compass_values: Compass sensor data
            lidar_values: LiDAR sensor data
            
        Returns:
            Tuple of (linear_velocity, angular_velocity) commands
        """
        # Process sensor data
        position = self.sensor_processor.process_gps_data(gps_values)
        heading = self.sensor_processor.process_compass_data(compass_values)
        lidar_data = self.sensor_processor.process_lidar_data(lidar_values)
        
        # Update state tracking
        self.last_position = position[:2]  # (x, y)
        self.last_heading = heading
        
        # Behavior coordination based on current mode
        if self.mode == NavigationMode.IDLE:
            return (0.0, 0.0)
            
        elif self.mode == NavigationMode.WAYPOINT_NAVIGATION:
            return self._handle_waypoint_navigation(position, heading, lidar_data)
            
        elif self.mode == NavigationMode.OBSTACLE_AVOIDANCE:
            return self._handle_obstacle_avoidance(position, heading, lidar_data)
            
        elif self.mode == NavigationMode.TARGET_APPROACH:
            return self._handle_target_approach(position, heading, lidar_data)
            
        elif self.mode == NavigationMode.GOAL_REACHED:
            return (0.0, 0.0)
        
        return (0.0, 0.0)
    
    def _handle_waypoint_navigation(self, position: Tuple[float, float, float],
                                   heading: float, lidar_data: Dict) -> Tuple[float, float]:
        """Handle waypoint-based navigation behavior."""
        # Check for obstacles that require avoidance
        if lidar_data['obstacles_detected'] and not lidar_data['front_clear']:
            return self._transition_to_obstacle_avoidance(position, heading, lidar_data)
        
        # Check if we've reached the current waypoint
        if self.current_waypoint_index < len(self.current_waypoints):
            current_waypoint = self.current_waypoints[self.current_waypoint_index]
            waypoint_pos = self.global_planner.get_coordinates(current_waypoint)
            
            distance_to_waypoint = math.sqrt(
                (position[0] - waypoint_pos[0])**2 + 
                (position[1] - waypoint_pos[1])**2
            )
            
            # Check if waypoint is reached
            if distance_to_waypoint < self.waypoint_tolerance:
                print(f"Reached waypoint: {current_waypoint}")
                self.current_waypoint_index += 1
                
                # Check if all waypoints completed
                if self.current_waypoint_index >= len(self.current_waypoints):
                    self.mode = NavigationMode.TARGET_APPROACH
                    return self._handle_target_approach(position, heading, lidar_data)
            
            # Navigate to current waypoint
            return self._navigate_to_position(position, heading, waypoint_pos)
        
        # No more waypoints, transition to target approach
        self.mode = NavigationMode.TARGET_APPROACH
        return self._handle_target_approach(position, heading, lidar_data)
    
    def _handle_obstacle_avoidance(self, position: Tuple[float, float, float],
                                  heading: float, lidar_data: Dict) -> Tuple[float, float]:
        """Handle obstacle avoidance behavior."""
        # Update obstacle avoidance with target memory
        if self.goal_position is not None:
            self.obstacle_avoidance.update_target_memory(
                self.goal_position, position[:2], heading
            )
        
        # Process avoidance behavior
        linear_vel, angular_vel, avoidance_state = self.obstacle_avoidance.process_avoidance(
            lidar_data, position[:2], heading
        )
        
        # Check if we can return to normal navigation
        if avoidance_state == AvoidanceState.NAVIGATING:
            self.mode = NavigationMode.WAYPOINT_NAVIGATION
            return self._handle_waypoint_navigation(position, heading, lidar_data)
        
        return (linear_vel, angular_vel)
    
    def _handle_target_approach(self, position: Tuple[float, float, float],
                               heading: float, lidar_data: Dict) -> Tuple[float, float]:
        """Handle final approach to target behavior."""
        if self.goal_position is None:
            self.mode = NavigationMode.IDLE
            return (0.0, 0.0)
        
        # Check for obstacles during approach
        if lidar_data['obstacles_detected'] and not lidar_data['front_clear']:
            return self._transition_to_obstacle_avoidance(position, heading, lidar_data)
        
        # Calculate distance to goal
        distance_to_goal = math.sqrt(
            (position[0] - self.goal_position[0])**2 + 
            (position[1] - self.goal_position[1])**2
        )
        
        # Check if goal is reached
        if distance_to_goal < self.goal_tolerance:
            print(f"Goal reached: {self.current_target}")
            self.mode = NavigationMode.GOAL_REACHED
            self.obstacle_avoidance.reset_target_memory()
            return (0.0, 0.0)
        
        # Navigate to goal with reduced speed for precision
        linear_vel, angular_vel = self._navigate_to_position(
            position, heading, self.goal_position
        )
        
        # Reduce speed for final approach
        linear_vel *= 0.6
        angular_vel *= 0.8
        
        return (linear_vel, angular_vel)
    
    def _transition_to_obstacle_avoidance(self, position: Tuple[float, float, float],
                                         heading: float, lidar_data: Dict) -> Tuple[float, float]:
        """Transition to obstacle avoidance mode."""
        print("Transitioning to obstacle avoidance")
        self.mode = NavigationMode.OBSTACLE_AVOIDANCE
        
        # Set target memory for obstacle avoidance
        if self.goal_position is not None:
            self.obstacle_avoidance.update_target_memory(
                self.goal_position, position[:2], heading
            )
        
        return self._handle_obstacle_avoidance(position, heading, lidar_data)
    
    def _navigate_to_position(self, current_pos: Tuple[float, float, float],
                             current_heading: float, 
                             target_pos: Tuple[float, float]) -> Tuple[float, float]:
        """
        Calculate control commands to navigate to a target position.
        
        Args:
            current_pos: Current robot position (x, y, z)
            current_heading: Current robot heading in radians
            target_pos: Target position (x, y)
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        # Calculate target direction
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:  # Very close to target
            return (0.0, 0.0)
        
        # Calculate target angle
        target_world_angle = math.atan2(dy, dx)
        angle_error = target_world_angle - current_heading
        
        # Normalize angle error to [-π, π]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Calculate control commands
        angular_vel = self.kp_angular * angle_error
        
        # Reduce linear velocity when turning
        angle_factor = max(0.1, 1.0 - abs(angle_error) / math.pi)
        linear_vel = self.kp_linear * distance * angle_factor
        
        # Apply velocity limits
        linear_vel = max(-self.max_linear_velocity, 
                        min(self.max_linear_velocity, linear_vel))
        angular_vel = max(-self.max_angular_velocity, 
                         min(self.max_angular_velocity, angular_vel))
        
        return (linear_vel, angular_vel)
    
    def is_navigation_complete(self) -> bool:
        """Check if current navigation task is complete."""
        return self.mode == NavigationMode.GOAL_REACHED
    
    def reset_navigation(self):
        """Reset navigation state for new task."""
        self.mode = NavigationMode.IDLE
        self.current_target = None
        self.current_waypoints = []
        self.current_waypoint_index = 0
        self.goal_position = None
        self.navigation_start_time = None
        self.obstacle_avoidance.reset_target_memory()
    
    def get_status_info(self) -> Dict:
        """
        Get current navigation status for debugging.
        
        Returns:
            Dictionary with current status information
        """
        return {
            'mode': self.mode.value,
            'target': self.current_target,
            'waypoints': self.current_waypoints,
            'waypoint_index': self.current_waypoint_index,
            'goal_position': self.goal_position,
            'avoidance_info': self.obstacle_avoidance.get_state_info()
        }
