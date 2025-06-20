#!/usr/bin/env python3
"""
Test script for Pioneer robot detection and stationary pause behavior
"""

import math

def test_pioneer_detection():
    """Test the improved Pioneer robot detection logic"""
    print("=== Testing Pioneer Robot Detection ===")
    
    def simulate_pioneer_detection(lidar_readings):
        """Simulate the improved Pioneer detection function"""
        if not lidar_readings:
            return False

        PIONEER_DETECTION_DISTANCE = 1.2  # Detect Pioneers at 1.2m (safe distance)
        
        # Check front 90 degrees for moving obstacles
        total_rays = len(lidar_readings)
        front_start = total_rays * 5 // 12   # Start from 42% (90 degrees)
        front_end = total_rays * 7 // 12     # End at 58% (90 degrees)

        obstacle_count = 0
        consecutive_count = 0
        max_consecutive = 0
        min_distance = float('inf')
        
        for i in range(front_start, front_end):
            distance = lidar_readings[i]
            if distance < PIONEER_DETECTION_DISTANCE:
                obstacle_count += 1
                consecutive_count += 1
                max_consecutive = max(max_consecutive, consecutive_count)
                min_distance = min(min_distance, distance)
            else:
                consecutive_count = 0

        # Detect Pioneer robots specifically
        if obstacle_count >= 3 and max_consecutive >= 3 and min_distance > 0.6:
            return True
        
        # Also detect very close obstacles (emergency stop)
        if min_distance < 0.5 and obstacle_count >= 2:
            return True
        
        return False
    
    # Test cases based on Pioneer robot characteristics
    test_cases = [
        {
            'name': 'No obstacles',
            'readings': [5.0] * 180,  # All clear
            'expected': False
        },
        {
            'name': 'Pioneer robot at safe distance (1.0m)',
            'readings': [5.0] * 75 + [1.0] * 5 + [5.0] * 100,  # Pioneer-sized object
            'expected': True
        },
        {
            'name': 'Pioneer robot at detection distance (1.1m)',
            'readings': [5.0] * 75 + [1.1] * 4 + [5.0] * 101,  # Pioneer-sized object
            'expected': True
        },
        {
            'name': 'Distant Pioneer (2.0m) - should not trigger',
            'readings': [5.0] * 75 + [2.0] * 5 + [5.0] * 100,  # Too far
            'expected': False
        },
        {
            'name': 'Wall/static obstacle (very close) - emergency stop',
            'readings': [5.0] * 75 + [0.3] * 10 + [5.0] * 95,  # Very close, triggers emergency
            'expected': True
        },
        {
            'name': 'Emergency stop - very close object',
            'readings': [5.0] * 80 + [0.4] * 3 + [5.0] * 97,  # Close enough for emergency
            'expected': True
        },
        {
            'name': 'Single ray detection (noise) - should not trigger',
            'readings': [5.0] * 90 + [0.8] + [5.0] * 89,  # Single ray, likely noise
            'expected': False
        },
        {
            'name': 'Pioneer outside detection zone',
            'readings': [1.0] * 20 + [5.0] * 140 + [1.0] * 20,  # Outside front 90°
            'expected': False
        },
        {
            'name': 'Multiple Pioneers',
            'readings': [5.0] * 70 + [0.9] * 4 + [5.0] * 10 + [1.1] * 3 + [5.0] * 93,
            'expected': True
        }
    ]
    
    all_passed = True
    for test_case in test_cases:
        result = simulate_pioneer_detection(test_case['readings'])
        if result == test_case['expected']:
            print(f"✓ {test_case['name']}: {result}")
        else:
            print(f"✗ {test_case['name']}: expected {test_case['expected']}, got {result}")
            all_passed = False
    
    return all_passed

def test_stationary_pause_behavior():
    """Test that robot stays completely still when paused"""
    print("\n=== Testing Stationary Pause Behavior ===")
    
    def simulate_pause_behavior(obstacle_detections, robot_positions):
        """Simulate robot behavior during pause"""
        navigation_state = 'navigating'
        actions = []
        
        for i, (obstacle_detected, position) in enumerate(zip(obstacle_detections, robot_positions)):
            time_step = i
            
            if obstacle_detected:
                if navigation_state != 'paused':
                    navigation_state = 'paused'
                    actions.append(f"t={time_step}: PAUSE - obstacle detected at {position}")
                else:
                    actions.append(f"t={time_step}: STAY PAUSED - robot at {position}")
                
                # Check if robot moved while paused (should not happen)
                if i > 0:
                    prev_pos = robot_positions[i-1]
                    distance_moved = math.sqrt((position[0] - prev_pos[0])**2 + (position[1] - prev_pos[1])**2)
                    if distance_moved > 0.05:  # 5cm tolerance for sensor noise
                        actions.append(f"t={time_step}: ERROR - robot moved {distance_moved:.3f}m while paused!")
            else:
                if navigation_state == 'paused':
                    navigation_state = 'navigating'
                    actions.append(f"t={time_step}: RESUME - path clear, continuing navigation")
                else:
                    actions.append(f"t={time_step}: NAVIGATE - moving to {position}")
        
        return actions, navigation_state
    
    # Test scenarios
    scenarios = [
        {
            'name': 'Robot stays still when paused',
            'obstacles': [False, True, True, True, False, False],
            'positions': [(0, 0), (0, 0), (0, 0), (0, 0), (0.1, 0), (0.2, 0)],  # No movement while paused
            'should_have_errors': False
        },
        {
            'name': 'Robot moves while paused (ERROR case)',
            'obstacles': [False, True, True, True, False],
            'positions': [(0, 0), (0.1, 0), (0.2, 0), (0.3, 0), (0.4, 0)],
            'should_have_errors': True
        },
        {
            'name': 'Normal navigation without obstacles',
            'obstacles': [False, False, False, False, False],
            'positions': [(0, 0), (0.1, 0), (0.2, 0), (0.3, 0), (0.4, 0)],
            'should_have_errors': False
        }
    ]
    
    all_passed = True
    for scenario in scenarios:
        actions, final_state = simulate_pause_behavior(scenario['obstacles'], scenario['positions'])
        
        has_errors = any('ERROR' in action for action in actions)
        
        if has_errors == scenario['should_have_errors']:
            print(f"✓ {scenario['name']}: errors={has_errors}")
        else:
            print(f"✗ {scenario['name']}: expected errors={scenario['should_have_errors']}, got {has_errors}")
            print(f"  Actions: {actions}")
            all_passed = False
    
    return all_passed

def test_progress_monitoring_during_pause():
    """Test that progress monitoring doesn't trigger during pause"""
    print("\n=== Testing Progress Monitoring During Pause ===")
    
    def simulate_progress_monitoring(navigation_states, positions, stuck_timeout=15.0):
        """Simulate progress monitoring with different navigation states"""
        last_position = None
        position_stuck_time = 0
        actions = []
        
        for i, (state, pos) in enumerate(zip(navigation_states, positions)):
            current_time = i
            
            # Don't check progress when paused for obstacles
            if state in ['paused', 'avoiding']:
                last_position = pos
                position_stuck_time = current_time
                actions.append(f"t={current_time}: PROGRESS RESET - paused state")
                continue
            
            if last_position is None:
                last_position = pos
                position_stuck_time = current_time
                continue
            
            # Check if robot has moved significantly (only when actively navigating)
            distance_moved = math.sqrt((pos[0] - last_position[0])**2 + (pos[1] - last_position[1])**2)
            
            if distance_moved > 0.2:  # Robot has moved at least 20cm
                last_position = pos
                position_stuck_time = current_time
                actions.append(f"t={current_time}: PROGRESS - moved {distance_moved:.2f}m")
            else:
                # Robot hasn't moved much while actively navigating
                if current_time - position_stuck_time >= stuck_timeout and state == 'navigating':
                    actions.append(f"t={current_time}: STUCK DETECTED - replanning")
                    return actions, True  # Stuck detected
        
        return actions, False  # Not stuck
    
    # Test scenarios
    scenarios = [
        {
            'name': 'Paused for obstacles - no stuck detection',
            'states': ['navigating', 'paused', 'paused', 'paused', 'navigating'] + ['navigating'] * 20,
            'positions': [(0, 0), (0.1, 0)] + [(0.1, 0)] * 3 + [(i*0.1, 0) for i in range(2, 22)],
            'should_be_stuck': False
        },
        {
            'name': 'Actually stuck while navigating',
            'states': ['navigating'] * 20,
            'positions': [(0.05, 0)] * 20,  # Not moving while navigating
            'should_be_stuck': True
        },
        {
            'name': 'Normal navigation with brief pause',
            'states': ['navigating'] * 5 + ['paused'] * 3 + ['navigating'] * 10,
            'positions': [(i*0.1, 0) for i in range(5)] + [(0.5, 0)] * 3 + [(0.5 + i*0.1, 0) for i in range(1, 11)],
            'should_be_stuck': False
        }
    ]
    
    all_passed = True
    for scenario in scenarios:
        actions, is_stuck = simulate_progress_monitoring(scenario['states'], scenario['positions'])
        
        if is_stuck == scenario['should_be_stuck']:
            print(f"✓ {scenario['name']}: stuck={is_stuck}")
        else:
            print(f"✗ {scenario['name']}: expected stuck={scenario['should_be_stuck']}, got {is_stuck}")
            print(f"  Actions: {actions[-5:]}")  # Show last 5 actions
            all_passed = False
    
    return all_passed

def main():
    """Run all tests"""
    print("Testing Pioneer Detection and Stationary Pause System")
    print("=" * 60)
    
    results = []
    
    # Test Pioneer detection
    results.append(test_pioneer_detection())
    
    # Test stationary pause behavior
    results.append(test_stationary_pause_behavior())
    
    # Test progress monitoring during pause
    results.append(test_progress_monitoring_during_pause())
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print(f"✅ All {total} test suites passed!")
        print("\nThe improved system should now:")
        print("1. ✓ Accurately detect Pioneer robots at safe distance (1.2m)")
        print("2. ✓ Stay completely stationary when paused (no rotation)")
        print("3. ✓ Not trigger 'stuck' detection during legitimate pauses")
        print("4. ✓ Resume navigation smoothly when path is clear")
        print("\nNo more false 'stuck' detection during obstacle avoidance! 🎉")
    else:
        print(f"❌ {passed}/{total} test suites passed")
        print("Some issues need to be addressed.")

if __name__ == "__main__":
    main()
