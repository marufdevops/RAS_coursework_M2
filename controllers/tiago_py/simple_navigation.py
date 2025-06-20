"""
Simplified Navigation System for Mission 2

A robust, simplified navigation system that focuses on basic functionality
without complex PDDL planning. Uses direct coordinate navigation with
obstacle avoidance.

Author: Mission 2 Navigation System
"""

import math
import time
from typing import Tuple, Optional, Dict, List
from enum import Enum


class NavigationState(Enum):
    """Navigation states."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    AVOIDING = "avoiding"
    GOAL_REACHED = "goal_reached"


class SimpleNavigator:
    """
    Simplified navigation system with direct coordinate navigation.
    
    Provides robust navigation to target locations with obstacle avoidance
    using a simplified approach that's easier to debug and maintain.
    """
    
    def __init__(self):
        """Initialize the simple navigator."""
        # Target locations (from goalchecker.py analysis)
        self.target_locations = {
            "red": (3.685, 3.01),      # Center of red box area
            "green": (3.685, -3.94),   # Center of green box area
            "ducks": (-3.07, 3.64),    # Center of ducks area
            "balls": (-2.795, -4.17)   # Center of balls area
        }
        
        # Navigation state
        self.state = NavigationState.IDLE
        self.current_target = None
        self.target_position = None
        
        # Control parameters
        self.goal_tolerance = 0.8  # Distance to consider goal reached
        self.obstacle_threshold = 0.4  # Critical obstacle distance
        self.safe_distance = 0.8  # Preferred minimum distance
        
        # Obstacle avoidance parameters
        self.avoidance_start_time = None
        self.max_avoidance_time = 8.0  # Maximum avoidance time
        self.min_avoidance_time = 1.0  # Minimum avoidance time
        
        # Control gains
        self.kp_linear = 1.0
        self.kp_angular = 2.0
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 0.8
        
    def set_target(self, target: str) -> bool:
        """
        Set navigation target.
        
        Args:
            target: Target name (red, green, ducks, balls)
            
        Returns:
            True if target is valid, False otherwise
        """
        if target not in self.target_locations:
            print(f"Invalid target: {target}")
            return False
            
        self.current_target = target
        self.target_position = self.target_locations[target]
        self.state = NavigationState.NAVIGATING
        
        print(f"Navigation target set: {target} at {self.target_position}")
        return True
    
    def update(self, gps_values: List[float], compass_values: List[float], 
               lidar_values: List[float]) -> Tuple[float, float]:
        """
        Main navigation update loop.
        
        Args:
            gps_values: GPS sensor data [x, y, z]
            compass_values: Compass sensor data [x, y, z]
            lidar_values: LiDAR range data
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        if self.state == NavigationState.IDLE:
            return (0.0, 0.0)
            
        # Process sensor data
        current_pos = (gps_values[0], gps_values[1])
        current_heading = math.atan2(compass_values[1], compass_values[0])
        obstacle_info = self._process_lidar(lidar_values)
        
        # Check if goal is reached
        if self.target_position:
            distance_to_goal = self._calculate_distance(current_pos, self.target_position)
            if distance_to_goal < self.goal_tolerance:
                self.state = NavigationState.GOAL_REACHED
                print(f"Goal reached: {self.current_target}")
                return (0.0, 0.0)
        
        # Navigation behavior based on state
        if self.state == NavigationState.NAVIGATING:
            return self._handle_navigation(current_pos, current_heading, obstacle_info)
        elif self.state == NavigationState.AVOIDING:
            return self._handle_avoidance(current_pos, current_heading, obstacle_info)
        
        return (0.0, 0.0)
    
    def _handle_navigation(self, current_pos: Tuple[float, float], 
                          current_heading: float, obstacle_info: Dict) -> Tuple[float, float]:
        """Handle normal navigation behavior."""
        # Check for obstacles requiring avoidance
        if obstacle_info['front_obstacle'] and obstacle_info['closest_distance'] < self.obstacle_threshold:
            print("Obstacle detected, switching to avoidance")
            self.state = NavigationState.AVOIDING
            self.avoidance_start_time = time.time()
            return self._handle_avoidance(current_pos, current_heading, obstacle_info)
        
        # Navigate toward target
        if self.target_position:
            return self._navigate_to_position(current_pos, current_heading, self.target_position)
        
        return (0.0, 0.0)
    
    def _handle_avoidance(self, current_pos: Tuple[float, float], 
                         current_heading: float, obstacle_info: Dict) -> Tuple[float, float]:
        """Handle obstacle avoidance behavior."""
        current_time = time.time()
        avoidance_duration = current_time - self.avoidance_start_time if self.avoidance_start_time else 0
        
        # Check if we can return to navigation
        if (avoidance_duration > self.min_avoidance_time and 
            not obstacle_info['front_obstacle']):
            print("Path clear, returning to navigation")
            self.state = NavigationState.NAVIGATING
            self.avoidance_start_time = None
            return self._handle_navigation(current_pos, current_heading, obstacle_info)
        
        # Check for avoidance timeout
        if avoidance_duration > self.max_avoidance_time:
            print("Avoidance timeout, returning to navigation")
            self.state = NavigationState.NAVIGATING
            self.avoidance_start_time = None
            return (0.0, 0.0)
        
        # Execute avoidance maneuver
        return self._execute_avoidance(obstacle_info)
    
    def _execute_avoidance(self, obstacle_info: Dict) -> Tuple[float, float]:
        """Execute obstacle avoidance maneuver."""
        # Simple avoidance strategy: turn away from closest obstacle
        if obstacle_info['closest_distance'] < self.obstacle_threshold:
            # Stop and turn
            linear_vel = 0.0
            
            # Turn left if obstacle is on right, right if obstacle is on left
            if obstacle_info['left_clear'] and not obstacle_info['right_clear']:
                angular_vel = 0.5  # Turn left
            elif obstacle_info['right_clear'] and not obstacle_info['left_clear']:
                angular_vel = -0.5  # Turn right
            else:
                # Default turn left
                angular_vel = 0.5
        else:
            # Move forward slowly while turning
            linear_vel = 0.3
            angular_vel = 0.3 if obstacle_info['left_clear'] else -0.3
        
        return (linear_vel, angular_vel)
    
    def _navigate_to_position(self, current_pos: Tuple[float, float], 
                             current_heading: float, 
                             target_pos: Tuple[float, float]) -> Tuple[float, float]:
        """Navigate to target position using proportional control."""
        # Calculate target direction
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:  # Very close
            return (0.0, 0.0)
        
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - current_heading
        
        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Calculate control commands
        angular_vel = self.kp_angular * angle_error
        
        # Reduce linear velocity when turning
        angle_factor = max(0.2, 1.0 - abs(angle_error) / math.pi)
        linear_vel = self.kp_linear * distance * angle_factor
        
        # Apply velocity limits
        linear_vel = max(-self.max_linear_velocity, 
                        min(self.max_linear_velocity, linear_vel))
        angular_vel = max(-self.max_angular_velocity, 
                         min(self.max_angular_velocity, angular_vel))
        
        return (linear_vel, angular_vel)
    
    def _process_lidar(self, lidar_values: List[float]) -> Dict:
        """Process LiDAR data for obstacle detection."""
        if not lidar_values:
            return {
                'front_obstacle': False,
                'left_clear': True,
                'right_clear': True,
                'closest_distance': float('inf')
            }
        
        # Divide scan into sectors
        num_readings = len(lidar_values)
        front_start = int(num_readings * 0.4)
        front_end = int(num_readings * 0.6)
        left_end = int(num_readings * 0.3)
        right_start = int(num_readings * 0.7)
        
        # Check each sector
        front_obstacle = False
        left_clear = True
        right_clear = True
        closest_distance = float('inf')
        
        for i, distance in enumerate(lidar_values):
            if distance < 0.04 or distance > 30.0:  # Invalid reading
                continue
                
            if distance < closest_distance:
                closest_distance = distance
            
            if i >= front_start and i < front_end:  # Front sector
                if distance < self.safe_distance:
                    front_obstacle = True
            elif i < left_end:  # Left sector
                if distance < self.safe_distance:
                    left_clear = False
            elif i >= right_start:  # Right sector
                if distance < self.safe_distance:
                    right_clear = False
        
        return {
            'front_obstacle': front_obstacle,
            'left_clear': left_clear,
            'right_clear': right_clear,
            'closest_distance': closest_distance
        }
    
    def _calculate_distance(self, pos1: Tuple[float, float], 
                           pos2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two positions."""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def is_goal_reached(self) -> bool:
        """Check if current goal is reached."""
        return self.state == NavigationState.GOAL_REACHED
    
    def reset(self):
        """Reset navigation state."""
        self.state = NavigationState.IDLE
        self.current_target = None
        self.target_position = None
        self.avoidance_start_time = None
    
    def get_status(self) -> Dict:
        """Get current navigation status."""
        return {
            'state': self.state.value,
            'target': self.current_target,
            'target_position': self.target_position
        }
