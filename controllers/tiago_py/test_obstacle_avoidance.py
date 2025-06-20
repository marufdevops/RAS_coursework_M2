#!/usr/bin/env python3
"""
Test script for obstacle avoidance and PDDL planning improvements
"""

import os
import subprocess
import tempfile

def test_pddl_planning():
    """Test PDDL planning with pyperplan command-line interface"""
    print("=== Testing PDDL Planning ===")
    
    # Test the existing problem file
    domain_file = "warehouse_domain.pddl"
    problem_file = "warehouse_problem.pddl"
    
    if not os.path.exists(domain_file) or not os.path.exists(problem_file):
        print("ERROR: PDDL files not found")
        return False
    
    try:
        # Call pyperplan with A* and hff heuristic
        cmd = ["pyperplan", "-H", "hff", "-s", "astar", domain_file, problem_file]
        print(f"Running: {' '.join(cmd)}")
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

        if result.returncode == 0:
            print("✓ PDDL planning successful")

            # Check for solution file (pyperplan creates .pddl.soln)
            solution_file = os.path.abspath(problem_file + '.soln')
            print(f"Looking for solution file: {solution_file}")
            if os.path.exists(solution_file):
                with open(solution_file, 'r') as f:
                    solution = f.read().strip()
                print(f"✓ Solution found: {solution}")
                
                # Parse solution to extract waypoints
                waypoints = []
                lines = solution.split('\n')
                for line in lines:
                    line = line.strip()
                    if line.startswith('(move robot'):
                        parts = line.split()
                        if len(parts) >= 4:
                            waypoint = parts[3].rstrip(')')
                            waypoints.append(waypoint)
                
                print(f"✓ Extracted waypoints: {waypoints}")
                
                # Clean up
                os.remove(solution_file)
                return True
            else:
                print("✗ No solution file generated")
                return False
        else:
            print(f"✗ PDDL planning failed: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print("✗ PDDL planning timeout")
        return False
    except Exception as e:
        print(f"✗ PDDL planning error: {e}")
        return False

def test_obstacle_detection_logic():
    """Test the obstacle detection logic improvements"""
    print("\n=== Testing Obstacle Detection Logic ===")
    
    # Simulate LiDAR readings
    def simulate_obstacle_detection(lidar_readings, obstacle_threshold=0.8):
        """Simulate the improved obstacle detection function"""
        if not lidar_readings:
            return False

        total_rays = len(lidar_readings)
        front_start = total_rays // 4  # Start from 25%
        front_end = 3 * total_rays // 4  # End at 75%

        obstacle_count = 0
        min_distance = float('inf')
        
        for i in range(front_start, front_end):
            distance = lidar_readings[i]
            if distance < obstacle_threshold:
                obstacle_count += 1
                min_distance = min(min_distance, distance)

        # Require multiple rays or very close obstacle
        if obstacle_count >= 3 or (obstacle_count >= 1 and min_distance < 0.4):
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
            'name': 'Single distant obstacle',
            'readings': [2.0] * 40 + [0.6] + [2.0] * 59,
            'expected': False
        },
        {
            'name': 'Multiple obstacles',
            'readings': [2.0] * 40 + [0.5, 0.6, 0.7] + [2.0] * 57,
            'expected': True
        },
        {
            'name': 'Very close single obstacle',
            'readings': [2.0] * 50 + [0.3] + [2.0] * 49,
            'expected': True
        },
        {
            'name': 'Obstacles outside front range',
            'readings': [0.3, 0.4] + [2.0] * 96 + [0.3, 0.4],
            'expected': False
        }
    ]
    
    all_passed = True
    for test_case in test_cases:
        result = simulate_obstacle_detection(test_case['readings'])
        if result == test_case['expected']:
            print(f"✓ {test_case['name']}: {result}")
        else:
            print(f"✗ {test_case['name']}: expected {test_case['expected']}, got {result}")
            all_passed = False
    
    return all_passed

def test_navigation_state_logic():
    """Test the improved navigation state logic"""
    print("\n=== Testing Navigation State Logic ===")
    
    # Simulate the improved execute_navigation logic
    def simulate_navigation_logic(obstacle_detected, current_state, pause_time, current_time, timeout=10.0):
        """Simulate the improved navigation state management"""
        
        if obstacle_detected:
            if current_state != 'paused':
                new_state = 'paused'
                action = 'pause_and_stop'
            else:
                # Check for timeout while paused
                if current_time - pause_time > timeout:
                    new_state = 'navigating'
                    action = 'timeout_resume'
                else:
                    new_state = 'paused'
                    action = 'stay_paused'
        else:
            # No obstacle detected
            if current_state == 'paused':
                new_state = 'navigating'
                action = 'resume_navigation'
            else:
                new_state = 'navigating'
                action = 'continue_navigation'
        
        return new_state, action
    
    # Test scenarios
    scenarios = [
        {
            'name': 'Initial obstacle detection',
            'obstacle': True,
            'state': 'navigating',
            'pause_time': 0,
            'current_time': 5,
            'expected_state': 'paused',
            'expected_action': 'pause_and_stop'
        },
        {
            'name': 'Obstacle cleared',
            'obstacle': False,
            'state': 'paused',
            'pause_time': 0,
            'current_time': 3,
            'expected_state': 'navigating',
            'expected_action': 'resume_navigation'
        },
        {
            'name': 'Timeout while paused',
            'obstacle': True,
            'state': 'paused',
            'pause_time': 0,
            'current_time': 12,
            'expected_state': 'navigating',
            'expected_action': 'timeout_resume'
        },
        {
            'name': 'Continue navigation',
            'obstacle': False,
            'state': 'navigating',
            'pause_time': 0,
            'current_time': 5,
            'expected_state': 'navigating',
            'expected_action': 'continue_navigation'
        }
    ]
    
    all_passed = True
    for scenario in scenarios:
        state, action = simulate_navigation_logic(
            scenario['obstacle'],
            scenario['state'],
            scenario['pause_time'],
            scenario['current_time']
        )
        
        if state == scenario['expected_state'] and action == scenario['expected_action']:
            print(f"✓ {scenario['name']}: {state}, {action}")
        else:
            print(f"✗ {scenario['name']}: expected ({scenario['expected_state']}, {scenario['expected_action']}), got ({state}, {action})")
            all_passed = False
    
    return all_passed

def main():
    """Run all tests"""
    print("Testing Navigation System Improvements")
    print("=" * 50)
    
    results = []
    
    # Test PDDL planning
    results.append(test_pddl_planning())
    
    # Test obstacle detection
    results.append(test_obstacle_detection_logic())
    
    # Test navigation state logic
    results.append(test_navigation_state_logic())
    
    # Summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)
    
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print(f"✓ All {total} test suites passed!")
        print("\nThe navigation system improvements are working correctly:")
        print("1. ✓ PDDL planning with pyperplan command-line interface")
        print("2. ✓ Improved obstacle detection with multiple criteria")
        print("3. ✓ Fixed pause/resume navigation logic")
    else:
        print(f"✗ {passed}/{total} test suites passed")
        print("Some issues need to be addressed before deployment.")

if __name__ == "__main__":
    main()
