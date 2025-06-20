"""
Sensor Data Processing Module

This module handles processing of sensor data from LiDAR, GPS, and compass
for environmental perception and navigation in the warehouse environment.

Author: Mission 2 Navigation System
"""

import math
import numpy as np
from typing import List, Tuple, Optional, Dict


class SensorProcessor:
    """
    Processes sensor data for environmental perception and navigation.
    
    Handles LiDAR obstacle detection, GPS localization, and compass
    orientation processing for the TiaGo robot navigation system.
    """
    
    def __init__(self):
        """Initialize the sensor processor with configuration parameters."""
        # LiDAR configuration (from world file analysis)
        self.lidar_fov = 3.5  # 200 degrees in radians
        self.lidar_resolution = 200  # horizontal resolution
        self.lidar_max_range = 30.0  # meters
        self.lidar_min_range = 0.04  # meters
        
        # Obstacle detection parameters
        self.obstacle_threshold = 0.4  # meters - critical distance
        self.safe_distance = 0.8  # meters - preferred minimum distance
        self.front_sector_angle = math.pi / 3  # 60 degrees front sector
        
        # Navigation parameters
        self.position_history = []
        self.max_history_length = 10
        
        # Compass calibration
        self.compass_offset = 0.0  # Calibration offset if needed
        
    def process_lidar_data(self, lidar_values: List[float]) -> Dict:
        """
        Process LiDAR range data for obstacle detection and navigation.
        
        Args:
            lidar_values: List of range measurements from LiDAR
            
        Returns:
            Dictionary containing processed LiDAR information:
            - obstacles_detected: Boolean indicating obstacle presence
            - front_clear: Boolean indicating if front path is clear
            - left_clear: Boolean indicating if left path is clear  
            - right_clear: Boolean indicating if right path is clear
            - closest_obstacle_distance: Distance to nearest obstacle
            - closest_obstacle_angle: Angle to nearest obstacle
            - obstacle_sectors: List of obstacle presence per sector
        """
        if not lidar_values:
            return self._empty_lidar_result()
        
        # Calculate angle step
        angle_step = self.lidar_fov / len(lidar_values)
        start_angle = -self.lidar_fov / 2
        
        # Initialize result structure
        result = {
            'obstacles_detected': False,
            'front_clear': True,
            'left_clear': True,
            'right_clear': True,
            'closest_obstacle_distance': float('inf'),
            'closest_obstacle_angle': 0.0,
            'obstacle_sectors': [],
            'front_distances': [],
            'left_distances': [],
            'right_distances': []
        }
        
        # Divide LiDAR data into sectors
        num_values = len(lidar_values)
        front_start = int(num_values * 0.4)  # Front 20% of scan
        front_end = int(num_values * 0.6)
        left_end = int(num_values * 0.3)
        right_start = int(num_values * 0.7)
        
        # Process each measurement
        for i, distance in enumerate(lidar_values):
            angle = start_angle + i * angle_step
            
            # Skip invalid readings
            if distance < self.lidar_min_range or distance > self.lidar_max_range:
                continue
                
            # Check for obstacles
            if distance < self.obstacle_threshold:
                result['obstacles_detected'] = True
                
                # Update closest obstacle
                if distance < result['closest_obstacle_distance']:
                    result['closest_obstacle_distance'] = distance
                    result['closest_obstacle_angle'] = angle
            
            # Categorize by sector
            if i < left_end:  # Left sector
                result['left_distances'].append(distance)
                if distance < self.safe_distance:
                    result['left_clear'] = False
                    
            elif i >= front_start and i < front_end:  # Front sector
                result['front_distances'].append(distance)
                if distance < self.safe_distance:
                    result['front_clear'] = False
                    
            elif i >= right_start:  # Right sector
                result['right_distances'].append(distance)
                if distance < self.safe_distance:
                    result['right_clear'] = False
        
        # Calculate sector statistics
        result['front_min_distance'] = min(result['front_distances']) if result['front_distances'] else float('inf')
        result['left_min_distance'] = min(result['left_distances']) if result['left_distances'] else float('inf')
        result['right_min_distance'] = min(result['right_distances']) if result['right_distances'] else float('inf')
        
        return result
    
    def process_gps_data(self, gps_values: List[float]) -> Tuple[float, float, float]:
        """
        Process GPS data for robot localization.
        
        Args:
            gps_values: GPS coordinates [x, y, z]
            
        Returns:
            Tuple of (x, y, z) coordinates in world frame
        """
        if len(gps_values) < 3:
            return (0.0, 0.0, 0.0)
        
        x, y, z = gps_values[0], gps_values[1], gps_values[2]
        
        # Add to position history for filtering
        self.position_history.append((x, y, z))
        if len(self.position_history) > self.max_history_length:
            self.position_history.pop(0)
        
        # Apply simple moving average filter
        if len(self.position_history) > 1:
            avg_x = sum(pos[0] for pos in self.position_history) / len(self.position_history)
            avg_y = sum(pos[1] for pos in self.position_history) / len(self.position_history)
            avg_z = sum(pos[2] for pos in self.position_history) / len(self.position_history)
            return (avg_x, avg_y, avg_z)
        
        return (x, y, z)
    
    def process_compass_data(self, compass_values: List[float]) -> float:
        """
        Process compass data for robot orientation.
        
        Args:
            compass_values: Compass readings [x, y, z] components
            
        Returns:
            Robot heading angle in radians (-π to π)
        """
        if len(compass_values) < 2:
            return 0.0
        
        # Calculate heading from compass x,y components
        heading = math.atan2(compass_values[1], compass_values[0])
        
        # Apply calibration offset if needed
        heading += self.compass_offset
        
        # Normalize to [-π, π]
        while heading > math.pi:
            heading -= 2 * math.pi
        while heading < -math.pi:
            heading += 2 * math.pi
            
        return heading
    
    def detect_dynamic_obstacles(self, current_lidar: List[float], 
                                previous_lidar: List[float] = None) -> List[Tuple[float, float]]:
        """
        Detect dynamic obstacles by comparing consecutive LiDAR scans.
        
        Args:
            current_lidar: Current LiDAR scan
            previous_lidar: Previous LiDAR scan for comparison
            
        Returns:
            List of (distance, angle) tuples for detected dynamic obstacles
        """
        if not previous_lidar or len(current_lidar) != len(previous_lidar):
            return []
        
        dynamic_obstacles = []
        angle_step = self.lidar_fov / len(current_lidar)
        start_angle = -self.lidar_fov / 2
        
        for i, (curr_dist, prev_dist) in enumerate(zip(current_lidar, previous_lidar)):
            # Skip invalid readings
            if (curr_dist < self.lidar_min_range or curr_dist > self.lidar_max_range or
                prev_dist < self.lidar_min_range or prev_dist > self.lidar_max_range):
                continue
            
            # Detect significant changes indicating movement
            distance_change = abs(curr_dist - prev_dist)
            if distance_change > 0.2 and curr_dist < 5.0:  # 20cm change within 5m
                angle = start_angle + i * angle_step
                dynamic_obstacles.append((curr_dist, angle))
        
        return dynamic_obstacles
    
    def calculate_safe_direction(self, lidar_data: Dict) -> float:
        """
        Calculate the safest direction for obstacle avoidance.
        
        Args:
            lidar_data: Processed LiDAR data dictionary
            
        Returns:
            Recommended steering angle in radians (positive = left, negative = right)
        """
        if not lidar_data['obstacles_detected']:
            return 0.0  # No steering needed
        
        # If front is blocked, choose left or right based on clearance
        if not lidar_data['front_clear']:
            if lidar_data['left_clear'] and not lidar_data['right_clear']:
                return math.pi / 4  # Turn left 45 degrees
            elif lidar_data['right_clear'] and not lidar_data['left_clear']:
                return -math.pi / 4  # Turn right 45 degrees
            elif lidar_data['left_clear'] and lidar_data['right_clear']:
                # Both sides clear, choose based on distances
                if lidar_data['left_min_distance'] > lidar_data['right_min_distance']:
                    return math.pi / 4  # Turn left
                else:
                    return -math.pi / 4  # Turn right
            else:
                # Both sides blocked, turn away from closest obstacle
                if lidar_data['closest_obstacle_angle'] > 0:
                    return -math.pi / 2  # Turn right
                else:
                    return math.pi / 2  # Turn left
        
        # Front is clear but obstacles detected, make minor adjustment
        if lidar_data['closest_obstacle_angle'] > 0:
            return -math.pi / 8  # Slight right turn
        else:
            return math.pi / 8  # Slight left turn
    
    def _empty_lidar_result(self) -> Dict:
        """Return empty LiDAR result structure."""
        return {
            'obstacles_detected': False,
            'front_clear': True,
            'left_clear': True,
            'right_clear': True,
            'closest_obstacle_distance': float('inf'),
            'closest_obstacle_angle': 0.0,
            'obstacle_sectors': [],
            'front_distances': [],
            'left_distances': [],
            'right_distances': [],
            'front_min_distance': float('inf'),
            'left_min_distance': float('inf'),
            'right_min_distance': float('inf')
        }
    
    def is_path_clear_to_target(self, target_angle: float, target_distance: float,
                               lidar_data: Dict) -> bool:
        """
        Check if path to target is clear of obstacles.
        
        Args:
            target_angle: Angle to target in radians
            target_distance: Distance to target in meters
            lidar_data: Processed LiDAR data
            
        Returns:
            True if path is clear, False otherwise
        """
        if not lidar_data['obstacles_detected']:
            return True
        
        # Check if any obstacles are in the path to target
        angle_tolerance = math.pi / 12  # 15 degrees tolerance
        
        if (abs(target_angle) < angle_tolerance and 
            lidar_data['closest_obstacle_distance'] < target_distance):
            return False
            
        return True
