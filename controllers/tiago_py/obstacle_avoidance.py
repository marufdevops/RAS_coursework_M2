"""
Reactive Obstacle Avoidance System

This module implements real-time obstacle detection and avoidance using LiDAR data
with target memory for return navigation behavior.

Author: Mission 2 Navigation System
"""

import math
import time
from typing import Tuple, Optional, Dict, List
from enum import Enum


class AvoidanceState(Enum):
    """States for the obstacle avoidance behavior."""
    NAVIGATING = "navigating"
    AVOIDING = "avoiding"
    RETURNING = "returning"
    STUCK = "stuck"


class ObstacleAvoidance:
    """
    Reactive obstacle avoidance system with target memory.
    
    Implements subsumption-based obstacle avoidance that maintains
    awareness of the original navigation target during avoidance maneuvers.
    """
    
    def __init__(self):
        """Initialize the obstacle avoidance system."""
        # Avoidance parameters
        self.obstacle_threshold = 0.4  # Critical distance (matching Pioneer behavior)
        self.safe_distance = 0.8  # Preferred minimum distance
        self.avoidance_speed = 1.0  # Speed during avoidance
        self.rotation_speed = 0.5  # Angular velocity for turning
        
        # State management
        self.state = AvoidanceState.NAVIGATING
        self.state_start_time = time.time()
        self.min_avoidance_time = 1.0  # Minimum time in avoidance state
        self.max_avoidance_time = 8.0  # Maximum time before giving up
        
        # Target memory for return behavior
        self.original_target = None
        self.target_position = None
        self.target_angle = None
        self.target_distance = None
        
        # Avoidance behavior parameters
        self.avoidance_direction = 1  # 1 for left, -1 for right
        self.stuck_threshold = 0.1  # Minimum movement to avoid stuck state
        self.stuck_time_limit = 5.0  # Time limit before declaring stuck
        
        # History for stuck detection
        self.position_history = []
        self.max_history_length = 10
        
    def update_target_memory(self, target_pos: Tuple[float, float], 
                           current_pos: Tuple[float, float], 
                           current_heading: float):
        """
        Update the target memory for return navigation.
        
        Args:
            target_pos: Target position (x, y)
            current_pos: Current robot position (x, y)
            current_heading: Current robot heading in radians
        """
        self.target_position = target_pos
        
        # Calculate target angle and distance
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        
        self.target_distance = math.sqrt(dx**2 + dy**2)
        target_world_angle = math.atan2(dy, dx)
        self.target_angle = target_world_angle - current_heading
        
        # Normalize angle to [-π, π]
        while self.target_angle > math.pi:
            self.target_angle -= 2 * math.pi
        while self.target_angle < -math.pi:
            self.target_angle += 2 * math.pi
    
    def process_avoidance(self, lidar_data: Dict, current_pos: Tuple[float, float],
                         current_heading: float) -> Tuple[float, float, AvoidanceState]:
        """
        Process obstacle avoidance behavior and return control commands.
        
        Args:
            lidar_data: Processed LiDAR data from sensor processor
            current_pos: Current robot position (x, y)
            current_heading: Current robot heading in radians
            
        Returns:
            Tuple of (linear_velocity, angular_velocity, current_state)
        """
        current_time = time.time()
        state_duration = current_time - self.state_start_time
        
        # Update position history for stuck detection
        self._update_position_history(current_pos)
        
        # State machine for obstacle avoidance
        if self.state == AvoidanceState.NAVIGATING:
            return self._handle_navigating_state(lidar_data, current_pos, current_heading)
            
        elif self.state == AvoidanceState.AVOIDING:
            return self._handle_avoiding_state(lidar_data, state_duration)
            
        elif self.state == AvoidanceState.RETURNING:
            return self._handle_returning_state(lidar_data, current_pos, current_heading)
            
        elif self.state == AvoidanceState.STUCK:
            return self._handle_stuck_state(lidar_data, state_duration)
        
        return (0.0, 0.0, self.state)
    
    def _handle_navigating_state(self, lidar_data: Dict, current_pos: Tuple[float, float],
                                current_heading: float) -> Tuple[float, float, AvoidanceState]:
        """Handle normal navigation state."""
        # Check for obstacles requiring avoidance
        if lidar_data['obstacles_detected'] and not lidar_data['front_clear']:
            # Transition to avoidance
            self._transition_to_state(AvoidanceState.AVOIDING)
            
            # Determine avoidance direction based on obstacle distribution
            if lidar_data['left_clear'] and not lidar_data['right_clear']:
                self.avoidance_direction = 1  # Turn left
            elif lidar_data['right_clear'] and not lidar_data['left_clear']:
                self.avoidance_direction = -1  # Turn right
            else:
                # Choose direction based on target position if available
                if self.target_angle is not None:
                    self.avoidance_direction = 1 if self.target_angle > 0 else -1
                else:
                    # Default to left turn
                    self.avoidance_direction = 1
            
            return self._execute_avoidance_maneuver(lidar_data)
        
        # No obstacles, continue normal navigation
        return (0.0, 0.0, self.state)
    
    def _handle_avoiding_state(self, lidar_data: Dict, 
                              state_duration: float) -> Tuple[float, float, AvoidanceState]:
        """Handle obstacle avoidance state."""
        # Check if we've been avoiding too long
        if state_duration > self.max_avoidance_time:
            self._transition_to_state(AvoidanceState.STUCK)
            return (0.0, 0.0, self.state)
        
        # Check if path is clear and we can return to navigation
        if (state_duration > self.min_avoidance_time and 
            lidar_data['front_clear'] and 
            not lidar_data['obstacles_detected']):
            
            if self.target_position is not None:
                self._transition_to_state(AvoidanceState.RETURNING)
                return (0.0, 0.0, self.state)
            else:
                self._transition_to_state(AvoidanceState.NAVIGATING)
                return (0.0, 0.0, self.state)
        
        # Continue avoidance maneuver
        return self._execute_avoidance_maneuver(lidar_data)
    
    def _handle_returning_state(self, lidar_data: Dict, current_pos: Tuple[float, float],
                               current_heading: float) -> Tuple[float, float, AvoidanceState]:
        """Handle return to target state."""
        # Update target angle based on current position
        if self.target_position is not None:
            self.update_target_memory(self.target_position, current_pos, current_heading)
        
        # Check for new obstacles
        if lidar_data['obstacles_detected'] and not lidar_data['front_clear']:
            self._transition_to_state(AvoidanceState.AVOIDING)
            return self._execute_avoidance_maneuver(lidar_data)
        
        # Check if we're close enough to target to resume normal navigation
        if self.target_distance is not None and self.target_distance < 1.0:
            self._transition_to_state(AvoidanceState.NAVIGATING)
            return (0.0, 0.0, self.state)
        
        # Return toward target
        if self.target_angle is not None:
            # Calculate return velocities
            angular_vel = self._calculate_angular_velocity(self.target_angle)
            linear_vel = self.avoidance_speed * 0.7  # Slower return speed
            
            return (linear_vel, angular_vel, self.state)
        
        # No target information, return to navigation
        self._transition_to_state(AvoidanceState.NAVIGATING)
        return (0.0, 0.0, self.state)
    
    def _handle_stuck_state(self, lidar_data: Dict, 
                           state_duration: float) -> Tuple[float, float, AvoidanceState]:
        """Handle stuck state with recovery behavior."""
        # Try recovery maneuvers
        if state_duration < 2.0:
            # First attempt: reverse
            return (-self.avoidance_speed * 0.5, 0.0, self.state)
        elif state_duration < 4.0:
            # Second attempt: rotate in place
            return (0.0, self.rotation_speed, self.state)
        elif state_duration < 6.0:
            # Third attempt: reverse and rotate
            return (-self.avoidance_speed * 0.3, self.rotation_speed * 0.5, self.state)
        else:
            # Give up and return to navigation
            self._transition_to_state(AvoidanceState.NAVIGATING)
            return (0.0, 0.0, self.state)
    
    def _execute_avoidance_maneuver(self, lidar_data: Dict) -> Tuple[float, float, AvoidanceState]:
        """
        Execute the actual avoidance maneuver.
        
        Args:
            lidar_data: Processed LiDAR data
            
        Returns:
            Tuple of (linear_velocity, angular_velocity, current_state)
        """
        # If very close obstacle, stop and rotate
        if lidar_data['closest_obstacle_distance'] < self.obstacle_threshold:
            linear_vel = 0.0
            angular_vel = self.rotation_speed * self.avoidance_direction
        else:
            # Move forward while turning
            linear_vel = self.avoidance_speed * 0.5
            angular_vel = self.rotation_speed * self.avoidance_direction * 0.7
        
        return (linear_vel, angular_vel, self.state)
    
    def _calculate_angular_velocity(self, target_angle: float) -> float:
        """
        Calculate angular velocity to reach target angle.
        
        Args:
            target_angle: Target angle in radians
            
        Returns:
            Angular velocity in rad/s
        """
        # Proportional controller for angle
        kp = 2.0  # Proportional gain
        angular_vel = kp * target_angle
        
        # Limit angular velocity
        max_angular_vel = self.rotation_speed
        angular_vel = max(-max_angular_vel, min(max_angular_vel, angular_vel))
        
        return angular_vel
    
    def _transition_to_state(self, new_state: AvoidanceState):
        """
        Transition to a new avoidance state.
        
        Args:
            new_state: New state to transition to
        """
        if new_state != self.state:
            print(f"Avoidance state transition: {self.state.value} -> {new_state.value}")
            self.state = new_state
            self.state_start_time = time.time()
    
    def _update_position_history(self, current_pos: Tuple[float, float]):
        """Update position history for stuck detection."""
        self.position_history.append(current_pos)
        if len(self.position_history) > self.max_history_length:
            self.position_history.pop(0)
    
    def _is_stuck(self) -> bool:
        """
        Check if robot appears to be stuck.
        
        Returns:
            True if robot appears stuck, False otherwise
        """
        if len(self.position_history) < self.max_history_length:
            return False
        
        # Calculate total movement over history
        total_movement = 0.0
        for i in range(1, len(self.position_history)):
            dx = self.position_history[i][0] - self.position_history[i-1][0]
            dy = self.position_history[i][1] - self.position_history[i-1][1]
            total_movement += math.sqrt(dx**2 + dy**2)
        
        return total_movement < self.stuck_threshold
    
    def reset_target_memory(self):
        """Reset target memory when reaching a goal."""
        self.target_position = None
        self.target_angle = None
        self.target_distance = None
        self._transition_to_state(AvoidanceState.NAVIGATING)
    
    def get_state_info(self) -> Dict:
        """
        Get current state information for debugging.
        
        Returns:
            Dictionary with current state information
        """
        return {
            'state': self.state.value,
            'avoidance_direction': self.avoidance_direction,
            'target_angle': self.target_angle,
            'target_distance': self.target_distance,
            'state_duration': time.time() - self.state_start_time
        }
