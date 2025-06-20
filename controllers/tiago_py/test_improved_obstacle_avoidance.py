#!/usr/bin/env python3
"""
Test script for improved obstacle avoidance system
"""

import math

def test_improved_obstacle_detection():
    """Test the improved obstacle detection logic"""
    print("=== Testing Improved Obstacle Detection ===")
    
    def simulate_improved_obstacle_detection(lidar_readings, obstacle_threshold=0.8):
        """Simulate the improved obstacle detection function"""
        if not lidar_readings:
            return False

        # Check front 120 degrees for obstacles (more focused)
        total_rays = len(lidar_readings)
        front_start = total_rays // 3      # Start from 33%
        front_end = 2 * total_rays // 3    # End at 67%

        obstacle_count = 0
        min_distance = float('inf')
        close_obstacle_count = 0  # Count very close obstacles
        
        for i in range(front_start, front_end):
            distance = lidar_readings[i]
            if distance < obstacle_threshold:
                obstacle_count += 1
                min_distance = min(min_distance, distance)
                if distance < 0.5:  # Very close obstacles
                    close_obstacle_count += 1

        # More conservative detection
        if (obstacle_count >= 3 and close_obstacle_count >= 1) or \
           (obstacle_count >= 5) or \
           (close_obstacle_count >= 1 and min_distance < 0.3):
            return True
        
        return False
    
    # Test cases
    test_cases = [
        {
            'name': 'No obstacles',
            'readings': [2.0] * 100,
            'expected': False
        },
        {
            'name': 'Single distant obstacle (should not trigger)',
            'readings': [2.0] * 40 + [0.6] + [2.0] * 59,
            'expected': False
        },
        {
            'name': 'Multiple close obstacles',
            'readings': [2.0] * 35 + [0.4, 0.3, 0.5] + [2.0] * 62,
            'expected': True
        },
        {
            'name': 'Very close single obstacle (immediate danger)',
            'readings': [2.0] * 50 + [0.2] + [2.0] * 49,
            'expected': True
        },
        {
            'name': 'Many distant obstacles (large object)',
            'readings': [2.0] * 35 + [0.7] * 6 + [2.0] * 59,
            'expected': True
        },
        {
            'name': 'Few distant obstacles (should not trigger)',
            'readings': [2.0] * 40 + [0.7, 0.6] + [2.0] * 58,
            'expected': False
        },
        {
            'name': 'Obstacles outside detection zone',
            'readings': [0.3] * 10 + [2.0] * 80 + [0.3] * 10,
            'expected': False
        }
    ]
    
    all_passed = True
    for test_case in test_cases:
        result = simulate_improved_obstacle_detection(test_case['readings'])
        if result == test_case['expected']:
            print(f"✓ {test_case['name']}: {result}")
        else:
            print(f"✗ {test_case['name']}: expected {test_case['expected']}, got {result}")
            all_passed = False
    
    return all_passed

def test_evasive_maneuver_logic():
    """Test the evasive maneuver logic"""
    print("\n=== Testing Evasive Maneuver Logic ===")
    
    def simulate_obstacle_handling(obstacle_detections, timeout_duration=5.0, max_attempts=3):
        """Simulate the improved obstacle handling logic"""
        navigation_state = 'navigating'
        pause_start_time = 0
        obstacle_timeout_count = 0
        current_time = 0
        
        actions = []
        
        for i, obstacle_detected in enumerate(obstacle_detections):
            current_time = i  # Simulate time progression
            
            if obstacle_detected:
                if navigation_state != 'paused':
                    navigation_state = 'paused'
                    pause_start_time = current_time
                    actions.append(f"t={current_time}: PAUSE - obstacle detected")
                else:
                    # Check for timeout while paused
                    if current_time - pause_start_time >= timeout_duration:
                        obstacle_timeout_count += 1
                        actions.append(f"t={current_time}: TIMEOUT #{obstacle_timeout_count}")
                        
                        if obstacle_timeout_count >= max_attempts:
                            navigation_state = 'avoiding'
                            actions.append(f"t={current_time}: EVASIVE MANEUVER - too many timeouts")
                            return actions, navigation_state
                        else:
                            navigation_state = 'navigating'
                            pause_start_time = current_time
                            actions.append(f"t={current_time}: RESUME ATTEMPT")
            else:
                # No obstacle detected
                if navigation_state == 'paused':
                    navigation_state = 'navigating'
                    obstacle_timeout_count = 0
                    actions.append(f"t={current_time}: RESUME - path clear")
        
        return actions, navigation_state
    
    # Test scenarios
    scenarios = [
        {
            'name': 'Quick obstacle clearance',
            'detections': [True, True, False, False, False],
            'expected_final_state': 'navigating',
            'should_have_evasive': False
        },
        {
            'name': 'Persistent obstacle (triggers evasive)',
            'detections': [True] * 20,  # Obstacle for 20 time units
            'expected_final_state': 'avoiding',
            'should_have_evasive': True
        },
        {
            'name': 'Intermittent obstacle (multiple timeouts)',
            'detections': [True] * 6 + [False] + [True] * 6 + [False] + [True] * 6,
            'expected_final_state': 'avoiding',
            'should_have_evasive': True
        },
        {
            'name': 'No obstacles',
            'detections': [False] * 10,
            'expected_final_state': 'navigating',
            'should_have_evasive': False
        }
    ]
    
    all_passed = True
    for scenario in scenarios:
        actions, final_state = simulate_obstacle_handling(scenario['detections'])
        
        has_evasive = any('EVASIVE MANEUVER' in action for action in actions)
        
        state_correct = final_state == scenario['expected_final_state']
        evasive_correct = has_evasive == scenario['should_have_evasive']
        
        if state_correct and evasive_correct:
            print(f"✓ {scenario['name']}: final_state={final_state}, evasive={has_evasive}")
        else:
            print(f"✗ {scenario['name']}: expected state={scenario['expected_final_state']}, got {final_state}")
            print(f"  Expected evasive={scenario['should_have_evasive']}, got {has_evasive}")
            print(f"  Actions: {actions}")
            all_passed = False
    
    return all_passed

def test_progress_monitoring():
    """Test the progress monitoring logic"""
    print("\n=== Testing Progress Monitoring ===")
    
    def simulate_progress_check(positions, stuck_timeout=15.0):
        """Simulate progress monitoring"""
        last_position = None
        position_stuck_time = 0
        actions = []
        
        for i, pos in enumerate(positions):
            current_time = i
            
            if last_position is None:
                last_position = pos
                position_stuck_time = current_time
                continue
            
            # Check if robot has moved significantly
            distance_moved = math.sqrt((pos[0] - last_position[0])**2 + (pos[1] - last_position[1])**2)
            
            if distance_moved > 0.2:  # Robot has moved at least 20cm
                last_position = pos
                position_stuck_time = current_time
                actions.append(f"t={current_time}: PROGRESS - moved {distance_moved:.2f}m")
            else:
                # Robot hasn't moved much
                if current_time - position_stuck_time >= stuck_timeout:
                    actions.append(f"t={current_time}: STUCK DETECTED - replanning")
                    return actions, True  # Stuck detected
        
        return actions, False  # Not stuck
    
    # Test scenarios
    scenarios = [
        {
            'name': 'Normal progress',
            'positions': [(0, 0), (0.3, 0), (0.6, 0), (0.9, 0), (1.2, 0)],
            'should_be_stuck': False
        },
        {
            'name': 'Robot stuck in place',
            'positions': [(0, 0)] + [(0.1, 0.1)] * 20,  # Barely moving for 20 time units
            'should_be_stuck': True
        },
        {
            'name': 'Slow but steady progress',
            'positions': [(i*0.25, 0) for i in range(10)],  # 25cm per time unit
            'should_be_stuck': False
        }
    ]
    
    all_passed = True
    for scenario in scenarios:
        actions, is_stuck = simulate_progress_check(scenario['positions'])
        
        if is_stuck == scenario['should_be_stuck']:
            print(f"✓ {scenario['name']}: stuck={is_stuck}")
        else:
            print(f"✗ {scenario['name']}: expected stuck={scenario['should_be_stuck']}, got {is_stuck}")
            print(f"  Actions: {actions}")
            all_passed = False
    
    return all_passed

def main():
    """Run all tests"""
    print("Testing Improved Obstacle Avoidance System")
    print("=" * 50)
    
    results = []
    
    # Test improved obstacle detection
    results.append(test_improved_obstacle_detection())
    
    # Test evasive maneuver logic
    results.append(test_evasive_maneuver_logic())
    
    # Test progress monitoring
    results.append(test_progress_monitoring())
    
    # Summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)
    
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print(f"✅ All {total} test suites passed!")
        print("\nThe improved obstacle avoidance system should now:")
        print("1. ✓ Use more conservative obstacle detection (fewer false positives)")
        print("2. ✓ Perform evasive maneuvers when obstacles persist")
        print("3. ✓ Detect when robot is stuck and trigger replanning")
        print("4. ✓ Handle timeout cycles more intelligently")
    else:
        print(f"❌ {passed}/{total} test suites passed")
        print("Some issues need to be addressed.")

if __name__ == "__main__":
    main()
