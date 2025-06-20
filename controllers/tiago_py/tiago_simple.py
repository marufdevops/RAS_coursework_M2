"""
Simplified TiaGo Navigation Controller for Mission 2

A robust, simplified navigation system that provides reliable navigation
to target locations with obstacle avoidance. This version focuses on
core functionality without complex dependencies.

Author: Mission 2 Navigation System
"""

import math
import time
from controller import Robot
from keyboardreader import KeyboardReader
from goalchecker import get_goals_in_range
from simple_navigation import SimpleNavigator


class SimpleTaskManager:
    """Simple task management for navigation commands."""
    
    def __init__(self, keyboard_reader):
        """Initialize task manager."""
        self.keyboard_reader = keyboard_reader
        self.task_queue = []
        self.current_task = None
        self.valid_targets = {'red', 'green', 'ducks', 'balls'}
        
    def process_input(self) -> str:
        """Process keyboard input and return next target."""
        # Get keyboard command
        command = self.keyboard_reader.get_command()
        
        if command:
            # Process command
            targets = [target.strip() for target in command.split(',')]
            for target in targets:
                if target in self.valid_targets:
                    self.task_queue.append(target)
                    print(f"Added task: {target}")
                elif target == 'clear':
                    self.task_queue.clear()
                    print("Task queue cleared")
                elif target == 'status':
                    print(f"Queue: {self.task_queue}, Current: {self.current_task}")
        
        # Return next task if no current task
        if not self.current_task and self.task_queue:
            self.current_task = self.task_queue.pop(0)
            return self.current_task
            
        return None
    
    def complete_current_task(self):
        """Mark current task as completed."""
        if self.current_task:
            print(f"Task completed: {self.current_task}")
            self.current_task = None
    
    def get_status(self):
        """Get task manager status."""
        return {
            'current_task': self.current_task,
            'queue_length': len(self.task_queue),
            'queue': self.task_queue
        }


# Initialize robot and sensors
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize sensors
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)

# Initialize motors
left_motor = robot.getDevice("wheel_left_joint")
right_motor = robot.getDevice("wheel_right_joint")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize navigation system
keyboard = KeyboardReader(timestep)
task_manager = SimpleTaskManager(keyboard)
navigator = SimpleNavigator()

# Robot physical parameters
wheel_radius = 0.0985  # meters
wheel_separation = 0.4044  # meters

def set_wheel_velocities(linear_vel: float, angular_vel: float):
    """Convert linear and angular velocities to wheel velocities."""
    # Calculate wheel linear velocities
    left_wheel_linear_vel = linear_vel - (angular_vel * wheel_separation / 2)
    right_wheel_linear_vel = linear_vel + (angular_vel * wheel_separation / 2)
    
    # Convert to angular velocities
    left_wheel_angular_vel = left_wheel_linear_vel / wheel_radius
    right_wheel_angular_vel = right_wheel_linear_vel / wheel_radius
    
    # Apply to motors
    left_motor.setVelocity(left_wheel_angular_vel)
    right_motor.setVelocity(right_wheel_angular_vel)

# System state
navigation_active = False
last_status_time = time.time()
status_interval = 5.0

print("=== TiaGo Simple Navigation System Initialized ===")
print("Commands: red, green, ducks, balls (or combinations like 'red,green')")
print("Special: clear (clear queue), status (show status)")
print("System ready...")

# Main control loop
while robot.step(timestep) != -1:
    try:
        # Get sensor data
        gps_values = gps.getValues()
        compass_values = compass.getValues()
        lidar_values = lidar.getRangeImage()
        
        # Process task management
        new_target = task_manager.process_input()
        
        # Start new navigation if available
        if new_target and not navigation_active:
            if navigator.set_target(new_target):
                navigation_active = True
                print(f"Starting navigation to: {new_target}")
            else:
                print(f"Failed to set target: {new_target}")
                task_manager.complete_current_task()
        
        # Update navigation
        if navigation_active:
            linear_vel, angular_vel = navigator.update(gps_values, compass_values, lidar_values)
            set_wheel_velocities(linear_vel, angular_vel)
            
            # Check if goal reached
            if navigator.is_goal_reached():
                print(f"Navigation completed successfully!")
                task_manager.complete_current_task()
                navigator.reset()
                navigation_active = False
                
                # Check goals in range
                goals_in_range = get_goals_in_range(gps_values[0], gps_values[1])
                if goals_in_range:
                    print(f"Robot is close to: {goals_in_range}")
        else:
            # Stop robot when not navigating
            set_wheel_velocities(0.0, 0.0)
        
        # Periodic status reporting
        current_time = time.time()
        if current_time - last_status_time > status_interval:
            last_status_time = current_time
            
            position = gps_values[:2]
            goals_nearby = get_goals_in_range(position[0], position[1])
            nav_status = navigator.get_status()
            task_status = task_manager.get_status()
            
            print(f"\n--- System Status ---")
            print(f"Position: ({position[0]:.2f}, {position[1]:.2f})")
            print(f"Navigation State: {nav_status['state']}")
            print(f"Current Target: {nav_status['target']}")
            print(f"Task Queue: {task_status['queue']}")
            print(f"Goals Nearby: {goals_nearby}")
            print("--------------------\n")
            
    except Exception as e:
        print(f"Error in main loop: {e}")
        # Stop robot on error
        set_wheel_velocities(0.0, 0.0)
        time.sleep(0.1)
