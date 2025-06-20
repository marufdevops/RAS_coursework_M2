"""
Navigation Controller

This module implements low-level motor control for smooth navigation,
turning, and precise positioning near targets for the TiaGo robot.

Author: Mission 2 Navigation System
"""

import math
import time
from typing import Tuple, Optional, List
from controller import Robot


class NavigationController:
    """
    Low-level navigation controller for TiaGo robot motor control.
    
    Provides smooth velocity control, acceleration limiting, and precise
    positioning capabilities for robust navigation behavior.
    """
    
    def __init__(self, robot: Robot):
        """
        Initialize the navigation controller.
        
        Args:
            robot: Webots Robot instance
        """
        self.robot = robot
        
        # Get motor devices
        self.left_motor = robot.getDevice("wheel_left_joint")
        self.right_motor = robot.getDevice("wheel_right_joint")
        
        # Set motors to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Robot physical parameters
        self.wheel_radius = 0.0985  # meters (from TiaGo specifications)
        self.wheel_separation = 0.4044  # meters (distance between wheels)
        
        # Control parameters
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.max_linear_acceleration = 0.5  # m/s²
        self.max_angular_acceleration = 1.0  # rad/s²
        
        # Current state
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.last_update_time = time.time()
        
        # Smoothing parameters
        self.velocity_filter_alpha = 0.8  # Low-pass filter coefficient
        self.emergency_stop = False
        
        # Performance monitoring
        self.command_history = []
        self.max_history_length = 100
        
    def set_velocity(self, linear_velocity: float, angular_velocity: float):
        """
        Set desired robot velocities with smoothing and limits.
        
        Args:
            linear_velocity: Desired linear velocity in m/s
            angular_velocity: Desired angular velocity in rad/s
        """
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Handle emergency stop
        if self.emergency_stop:
            linear_velocity = 0.0
            angular_velocity = 0.0
        
        # Apply velocity limits
        linear_velocity = self._limit_velocity(
            linear_velocity, self.max_linear_velocity
        )
        angular_velocity = self._limit_velocity(
            angular_velocity, self.max_angular_velocity
        )
        
        # Apply acceleration limits
        if dt > 0:
            linear_velocity = self._limit_acceleration(
                linear_velocity, self.current_linear_velocity,
                self.max_linear_acceleration, dt
            )
            angular_velocity = self._limit_acceleration(
                angular_velocity, self.current_angular_velocity,
                self.max_angular_acceleration, dt
            )
        
        # Apply velocity smoothing
        self.current_linear_velocity = self._smooth_velocity(
            linear_velocity, self.current_linear_velocity
        )
        self.current_angular_velocity = self._smooth_velocity(
            angular_velocity, self.current_angular_velocity
        )
        
        # Convert to wheel velocities and apply
        left_wheel_vel, right_wheel_vel = self._differential_drive_kinematics(
            self.current_linear_velocity, self.current_angular_velocity
        )
        
        self.left_motor.setVelocity(left_wheel_vel)
        self.right_motor.setVelocity(right_wheel_vel)
        
        # Store command history
        self._update_command_history(
            self.current_linear_velocity, self.current_angular_velocity
        )
    
    def stop(self):
        """Immediately stop the robot."""
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
    
    def emergency_stop_enable(self):
        """Enable emergency stop mode."""
        self.emergency_stop = True
        self.stop()
    
    def emergency_stop_disable(self):
        """Disable emergency stop mode."""
        self.emergency_stop = False
    
    def rotate_to_heading(self, target_heading: float, current_heading: float,
                         angular_speed: float = 0.5) -> Tuple[float, float]:
        """
        Calculate velocities to rotate to a target heading.
        
        Args:
            target_heading: Target heading in radians
            current_heading: Current heading in radians
            angular_speed: Desired angular speed in rad/s
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        # Calculate angle error
        angle_error = target_heading - current_heading
        
        # Normalize angle error to [-π, π]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Proportional controller for rotation
        kp = 2.0
        angular_velocity = kp * angle_error
        
        # Limit angular velocity
        angular_velocity = max(-angular_speed, min(angular_speed, angular_velocity))
        
        # No linear motion during rotation
        linear_velocity = 0.0
        
        return (linear_velocity, angular_velocity)
    
    def move_to_position(self, target_pos: Tuple[float, float], 
                        current_pos: Tuple[float, float],
                        current_heading: float,
                        approach_speed: float = 0.5) -> Tuple[float, float]:
        """
        Calculate velocities to move to a target position.
        
        Args:
            target_pos: Target position (x, y)
            current_pos: Current position (x, y)
            current_heading: Current heading in radians
            approach_speed: Desired approach speed in m/s
            
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        # Calculate distance and angle to target
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.05:  # Very close to target
            return (0.0, 0.0)
        
        # Calculate target heading
        target_heading = math.atan2(dy, dx)
        angle_error = target_heading - current_heading
        
        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Control parameters
        kp_linear = 1.0
        kp_angular = 2.0
        
        # Calculate velocities
        angular_velocity = kp_angular * angle_error
        
        # Reduce linear velocity when turning
        angle_factor = max(0.1, 1.0 - abs(angle_error) / math.pi)
        linear_velocity = kp_linear * distance * angle_factor * approach_speed
        
        return (linear_velocity, angular_velocity)
    
    def precise_positioning(self, target_pos: Tuple[float, float],
                           current_pos: Tuple[float, float],
                           current_heading: float,
                           tolerance: float = 0.1) -> Tuple[float, float, bool]:
        """
        Perform precise positioning near target with fine control.
        
        Args:
            target_pos: Target position (x, y)
            current_pos: Current position (x, y)
            current_heading: Current heading in radians
            tolerance: Position tolerance in meters
            
        Returns:
            Tuple of (linear_velocity, angular_velocity, position_reached)
        """
        # Calculate distance to target
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if position is reached
        if distance < tolerance:
            return (0.0, 0.0, True)
        
        # Use reduced speed for precision
        precision_speed = 0.2
        linear_vel, angular_vel = self.move_to_position(
            target_pos, current_pos, current_heading, precision_speed
        )
        
        # Further reduce velocities for fine control
        linear_vel *= 0.5
        angular_vel *= 0.7
        
        return (linear_vel, angular_vel, False)
    
    def _differential_drive_kinematics(self, linear_vel: float, 
                                     angular_vel: float) -> Tuple[float, float]:
        """
        Convert linear and angular velocities to wheel velocities.
        
        Args:
            linear_vel: Linear velocity in m/s
            angular_vel: Angular velocity in rad/s
            
        Returns:
            Tuple of (left_wheel_velocity, right_wheel_velocity) in rad/s
        """
        # Calculate wheel linear velocities
        left_wheel_linear_vel = linear_vel - (angular_vel * self.wheel_separation / 2)
        right_wheel_linear_vel = linear_vel + (angular_vel * self.wheel_separation / 2)
        
        # Convert to angular velocities
        left_wheel_angular_vel = left_wheel_linear_vel / self.wheel_radius
        right_wheel_angular_vel = right_wheel_linear_vel / self.wheel_radius
        
        return (left_wheel_angular_vel, right_wheel_angular_vel)
    
    def _limit_velocity(self, velocity: float, max_velocity: float) -> float:
        """Apply velocity limits."""
        return max(-max_velocity, min(max_velocity, velocity))
    
    def _limit_acceleration(self, target_vel: float, current_vel: float,
                           max_acceleration: float, dt: float) -> float:
        """Apply acceleration limits."""
        max_delta_vel = max_acceleration * dt
        velocity_change = target_vel - current_vel
        
        if abs(velocity_change) > max_delta_vel:
            if velocity_change > 0:
                return current_vel + max_delta_vel
            else:
                return current_vel - max_delta_vel
        
        return target_vel
    
    def _smooth_velocity(self, target_vel: float, current_vel: float) -> float:
        """Apply velocity smoothing filter."""
        return (self.velocity_filter_alpha * target_vel + 
                (1 - self.velocity_filter_alpha) * current_vel)
    
    def _update_command_history(self, linear_vel: float, angular_vel: float):
        """Update command history for monitoring."""
        self.command_history.append({
            'timestamp': time.time(),
            'linear_velocity': linear_vel,
            'angular_velocity': angular_vel
        })
        
        if len(self.command_history) > self.max_history_length:
            self.command_history.pop(0)
    
    def get_current_velocities(self) -> Tuple[float, float]:
        """
        Get current robot velocities.
        
        Returns:
            Tuple of (linear_velocity, angular_velocity)
        """
        return (self.current_linear_velocity, self.current_angular_velocity)
    
    def is_moving(self, threshold: float = 0.01) -> bool:
        """
        Check if robot is currently moving.
        
        Args:
            threshold: Velocity threshold for movement detection
            
        Returns:
            True if robot is moving, False otherwise
        """
        return (abs(self.current_linear_velocity) > threshold or
                abs(self.current_angular_velocity) > threshold)
    
    def get_performance_stats(self) -> dict:
        """
        Get performance statistics.
        
        Returns:
            Dictionary with performance metrics
        """
        if not self.command_history:
            return {}
        
        recent_commands = self.command_history[-10:]  # Last 10 commands
        
        avg_linear_vel = sum(cmd['linear_velocity'] for cmd in recent_commands) / len(recent_commands)
        avg_angular_vel = sum(cmd['angular_velocity'] for cmd in recent_commands) / len(recent_commands)
        
        return {
            'current_linear_velocity': self.current_linear_velocity,
            'current_angular_velocity': self.current_angular_velocity,
            'average_linear_velocity': avg_linear_vel,
            'average_angular_velocity': avg_angular_vel,
            'emergency_stop_active': self.emergency_stop,
            'command_history_length': len(self.command_history)
        }
