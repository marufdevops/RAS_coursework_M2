#!/usr/bin/env python3
"""
Test script to verify LiDAR sensor is working properly
"""

import math
import sys
import os

# Add the parent directory to the path to import controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from controller import Robot
except ImportError:
    print("This script must be run from within Webots")
    sys.exit(1)

def test_lidar_sensor():
    """Test LiDAR sensor functionality"""
    print("=== LiDAR Sensor Test ===")
    
    # Initialize robot
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # Get LiDAR device
    lidar = robot.getDevice("lidar")
    if lidar is None:
        print("❌ ERROR: LiDAR device 'lidar' not found!")
        return False
    
    # Enable LiDAR
    lidar.enable(timestep)
    
    # Get GPS for position
    gps = robot.getDevice('gps')
    if gps is None:
        print("❌ ERROR: GPS device not found!")
        return False
    
    gps.enable(timestep)
    
    print("✅ Devices initialized successfully")
    print("⏳ Waiting for sensor data...")
    
    # Wait for sensors to initialize
    for i in range(10):
        if robot.step(timestep) == -1:
            print("❌ ERROR: Robot step failed")
            return False
        
        lidar_values = lidar.getRangeImage()
        gps_values = gps.getValues()
        
        if lidar_values and len(lidar_values) > 0 and not (math.isnan(gps_values[0]) or math.isnan(gps_values[1])):
            break
    else:
        print("❌ ERROR: Sensors failed to initialize after 10 steps")
        return False
    
    print("✅ Sensors initialized")
    
    # Test LiDAR readings
    test_count = 0
    max_tests = 50  # Run for 50 timesteps
    
    while robot.step(timestep) != -1 and test_count < max_tests:
        test_count += 1
        
        # Get sensor data
        lidar_values = lidar.getRangeImage()
        gps_values = gps.getValues()
        
        if not lidar_values or len(lidar_values) == 0:
            print(f"❌ Step {test_count}: No LiDAR data")
            continue
        
        # Analyze LiDAR data
        total_rays = len(lidar_values)
        valid_readings = 0
        min_distance = float('inf')
        max_distance = 0
        center_ray = total_rays // 2
        
        for distance in lidar_values:
            if not (math.isnan(distance) or math.isinf(distance)):
                valid_readings += 1
                min_distance = min(min_distance, distance)
                max_distance = max(max_distance, distance)
        
        center_distance = lidar_values[center_ray] if center_ray < len(lidar_values) else float('nan')
        
        # Print summary every 10 steps
        if test_count % 10 == 0:
            print(f"\n📊 Step {test_count} Summary:")
            print(f"   Position: ({gps_values[0]:.2f}, {gps_values[1]:.2f})")
            print(f"   Total rays: {total_rays}")
            print(f"   Valid readings: {valid_readings}/{total_rays}")
            print(f"   Distance range: {min_distance:.2f}m - {max_distance:.2f}m")
            print(f"   Center ray distance: {center_distance:.2f}m")
            
            # Check front area for obstacles
            front_start = total_rays * 2 // 5  # 40% mark
            front_end = total_rays * 3 // 5    # 60% mark
            
            obstacles_detected = 0
            for i in range(front_start, front_end):
                if i < len(lidar_values) and lidar_values[i] < 2.0:  # Within 2m
                    obstacles_detected += 1
            
            print(f"   Front obstacles (< 2m): {obstacles_detected} rays")
            
            # Test obstacle detection function
            obstacle_detected = test_obstacle_detection(lidar_values)
            print(f"   Obstacle detection result: {obstacle_detected}")
    
    print(f"\n✅ LiDAR test completed successfully!")
    print(f"   Ran for {test_count} timesteps")
    print(f"   Final ray count: {len(lidar_values) if lidar_values else 0}")
    
    return True

def test_obstacle_detection(lidar_values):
    """Test the obstacle detection logic"""
    if not lidar_values or len(lidar_values) == 0:
        return False

    PIONEER_DETECTION_DISTANCE = 1.5
    
    # Check front 120 degrees
    total_rays = len(lidar_values)
    center_ray = total_rays // 2
    rays_per_degree = total_rays / 200  # TiaGo has 200° field of view
    half_front_rays = int((120 / 2) * rays_per_degree)
    
    front_start = max(0, center_ray - half_front_rays)
    front_end = min(total_rays, center_ray + half_front_rays)

    obstacle_count = 0
    consecutive_count = 0
    max_consecutive = 0
    min_distance = float('inf')
    close_obstacles = 0
    
    for i in range(front_start, front_end):
        if i >= len(lidar_values):
            break
            
        distance = lidar_values[i]
        
        if math.isnan(distance) or math.isinf(distance):
            consecutive_count = 0
            continue
            
        if distance < PIONEER_DETECTION_DISTANCE:
            obstacle_count += 1
            consecutive_count += 1
            max_consecutive = max(max_consecutive, consecutive_count)
            min_distance = min(min_distance, distance)
            
            if distance < 0.8:
                close_obstacles += 1
        else:
            consecutive_count = 0

    # Detection criteria
    pioneer_detected = (obstacle_count >= 4 and max_consecutive >= 3 and 
                       min_distance > 0.5 and min_distance < PIONEER_DETECTION_DISTANCE)
    
    emergency_stop = (close_obstacles >= 3 and min_distance < 0.6)
    
    return pioneer_detected or emergency_stop

def main():
    """Main test function"""
    try:
        success = test_lidar_sensor()
        if success:
            print("\n🎉 All tests passed!")
        else:
            print("\n❌ Tests failed!")
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
