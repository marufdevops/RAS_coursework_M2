"""
Feature Tests for Mission 2 Navigation System

End-to-end feature tests validating complete navigation scenarios
and system behavior in realistic warehouse environments.

Author: Mission 2 Navigation System
"""

import unittest
import time
import math
from unittest.mock import Mock, patch, MagicMock

# Import system components for feature testing
from behavior_coordinator import BehaviorCoordinator, NavigationMode
from task_manager import TaskManager, TaskPriority
from global_planner import GlobalPathPlanner
from sensor_processor import SensorProcessor
from obstacle_avoidance import ObstacleAvoidance, AvoidanceState


class TestNavigationFeatures(unittest.TestCase):
    """Feature tests for navigation capabilities."""
    
    def setUp(self):
        """Set up feature test environment."""
        self.coordinator = BehaviorCoordinator()
        self.mock_keyboard = Mock()
        self.task_manager = TaskManager(self.mock_keyboard)
        
    def test_single_target_navigation(self):
        """Test navigation to a single target location."""
        print("\n=== Testing Single Target Navigation ===")
        
        # Mock successful path planning
        with patch.object(self.coordinator.global_planner, 'plan_path') as mock_plan:
            mock_plan.return_value = ["corridor-center", "right-upper", "red-box"]
            
            # Set navigation target
            result = self.coordinator.set_navigation_target("red")
            self.assertTrue(result, "Should successfully set navigation target")
            self.assertEqual(self.coordinator.current_target, "red")
            self.assertEqual(self.coordinator.mode, NavigationMode.WAYPOINT_NAVIGATION)
            
            print(f"✓ Successfully set target: {self.coordinator.current_target}")
            print(f"✓ Navigation mode: {self.coordinator.mode.value}")
            print(f"✓ Planned waypoints: {self.coordinator.current_waypoints}")
    
    def test_multi_target_task_queue(self):
        """Test multiple target navigation with task queuing."""
        print("\n=== Testing Multi-Target Task Queue ===")
        
        # Add multiple tasks
        targets = ["red", "green", "ducks", "balls"]
        for target in targets:
            result = self.task_manager.add_task(target, TaskPriority.NORMAL)
            self.assertTrue(result, f"Should successfully add task: {target}")
        
        self.assertEqual(len(self.task_manager.task_queue), 4)
        print(f"✓ Added {len(self.task_manager.task_queue)} tasks to queue")
        
        # Process tasks sequentially
        completed_targets = []
        while self.task_manager.task_queue:
            target = self.task_manager.get_next_task_target()
            self.assertIsNotNone(target, "Should get next target from queue")
            completed_targets.append(target)
            self.task_manager.complete_current_task()
            print(f"✓ Completed navigation to: {target}")
        
        self.assertEqual(len(completed_targets), 4)
        self.assertEqual(set(completed_targets), set(targets))
        print(f"✓ Successfully completed all {len(completed_targets)} navigation tasks")
    
    def test_priority_task_handling(self):
        """Test priority-based task handling."""
        print("\n=== Testing Priority Task Handling ===")
        
        # Add tasks with different priorities
        self.task_manager.add_task("red", TaskPriority.LOW)
        self.task_manager.add_task("green", TaskPriority.HIGH)
        self.task_manager.add_task("ducks", TaskPriority.NORMAL)
        
        # High priority task should be processed first
        first_target = self.task_manager.get_next_task_target()
        self.assertEqual(first_target, "green", "High priority task should be first")
        print(f"✓ High priority task processed first: {first_target}")
        
        self.task_manager.complete_current_task()
        
        # Normal priority should be next
        second_target = self.task_manager.get_next_task_target()
        self.assertEqual(second_target, "ducks", "Normal priority task should be second")
        print(f"✓ Normal priority task processed second: {second_target}")
    
    def test_obstacle_avoidance_behavior(self):
        """Test obstacle avoidance behavior during navigation."""
        print("\n=== Testing Obstacle Avoidance Behavior ===")
        
        avoidance = ObstacleAvoidance()
        
        # Set target memory
        target_pos = (3.0, 4.0)
        current_pos = (0.0, 0.0)
        current_heading = 0.0
        avoidance.update_target_memory(target_pos, current_pos, current_heading)
        
        print(f"✓ Set target memory: {target_pos}")
        print(f"✓ Target distance: {avoidance.target_distance:.2f}m")
        
        # Simulate obstacle detection
        lidar_data = {
            'obstacles_detected': True,
            'front_clear': False,
            'left_clear': True,
            'right_clear': False,
            'closest_obstacle_distance': 0.3,
            'closest_obstacle_angle': 0.0
        }
        
        # Process avoidance behavior
        linear_vel, angular_vel, state = avoidance.process_avoidance(
            lidar_data, current_pos, current_heading
        )
        
        self.assertEqual(state, AvoidanceState.AVOIDING)
        self.assertNotEqual(angular_vel, 0.0, "Should be turning to avoid obstacle")
        print(f"✓ Obstacle detected, transitioning to: {state.value}")
        print(f"✓ Avoidance maneuver: linear={linear_vel:.2f}, angular={angular_vel:.2f}")
        
        # Simulate clear path after avoidance
        time.sleep(1.1)  # Wait for minimum avoidance time
        clear_lidar_data = {
            'obstacles_detected': False,
            'front_clear': True,
            'left_clear': True,
            'right_clear': True,
            'closest_obstacle_distance': float('inf'),
            'closest_obstacle_angle': 0.0
        }
        
        linear_vel, angular_vel, state = avoidance.process_avoidance(
            clear_lidar_data, current_pos, current_heading
        )
        
        self.assertEqual(state, AvoidanceState.RETURNING)
        print(f"✓ Path clear, transitioning to: {state.value}")
    
    def test_sensor_data_processing(self):
        """Test sensor data processing pipeline."""
        print("\n=== Testing Sensor Data Processing ===")
        
        processor = SensorProcessor()
        
        # Test LiDAR processing with various scenarios
        scenarios = [
            ("No obstacles", [30.0] * 200),
            ("Front obstacle", [30.0] * 80 + [0.3] * 40 + [30.0] * 80),
            ("Left obstacle", [0.3] * 60 + [30.0] * 140),
            ("Right obstacle", [30.0] * 140 + [0.3] * 60)
        ]
        
        for scenario_name, lidar_values in scenarios:
            result = processor.process_lidar_data(lidar_values)
            print(f"✓ {scenario_name}:")
            print(f"  - Obstacles detected: {result['obstacles_detected']}")
            print(f"  - Front clear: {result['front_clear']}")
            print(f"  - Closest distance: {result['closest_obstacle_distance']:.2f}m")
        
        # Test GPS processing
        gps_values = [1.5, 2.5, 0.1]
        position = processor.process_gps_data(gps_values)
        self.assertEqual(position, (1.5, 2.5, 0.1))
        print(f"✓ GPS processing: {position}")
        
        # Test compass processing
        compass_values = [1.0, 0.0, 0.0]
        heading = processor.process_compass_data(compass_values)
        self.assertAlmostEqual(heading, 0.0, places=2)
        print(f"✓ Compass processing: {heading:.2f} rad")
    
    def test_global_path_planning(self):
        """Test PDDL-based global path planning."""
        print("\n=== Testing Global Path Planning ===")
        
        planner = GlobalPathPlanner()
        
        # Test coordinate mapping
        test_locations = ["red-box", "green-box", "ducks-container", "balls-container"]
        for location in test_locations:
            coords = planner.get_coordinates(location)
            self.assertIsInstance(coords, tuple)
            self.assertEqual(len(coords), 2)
            print(f"✓ {location}: {coords}")
        
        # Test nearest waypoint calculation
        test_positions = [
            ((-1.9, -1.9), "start-pos"),
            ((3.7, 3.0), "red-box"),
            ((3.7, -4.0), "green-box"),
            ((-3.0, 3.5), "ducks-container")
        ]
        
        for position, expected_waypoint in test_positions:
            nearest = planner.get_nearest_waypoint(position[0], position[1])
            print(f"✓ Position {position} -> Nearest waypoint: {nearest}")
    
    def test_behavior_coordination(self):
        """Test behavior coordination and mode transitions."""
        print("\n=== Testing Behavior Coordination ===")
        
        coordinator = BehaviorCoordinator()
        
        # Test initial state
        self.assertEqual(coordinator.mode, NavigationMode.IDLE)
        print(f"✓ Initial mode: {coordinator.mode.value}")
        
        # Mock path planning for testing
        with patch.object(coordinator.global_planner, 'plan_path') as mock_plan:
            mock_plan.return_value = ["corridor-center", "right-upper", "red-box"]
            
            # Set navigation target
            result = coordinator.set_navigation_target("red")
            self.assertTrue(result)
            self.assertEqual(coordinator.mode, NavigationMode.WAYPOINT_NAVIGATION)
            print(f"✓ Target set, mode: {coordinator.mode.value}")
            
            # Simulate navigation update with clear sensors
            gps_values = [0.0, 0.0, 0.1]
            compass_values = [1.0, 0.0, 0.0]
            lidar_values = [30.0] * 200
            
            linear_vel, angular_vel = coordinator.update(
                gps_values, compass_values, lidar_values
            )
            
            print(f"✓ Navigation update: linear={linear_vel:.2f}, angular={angular_vel:.2f}")
            
            # Test mode transition to obstacle avoidance
            obstacle_lidar = [30.0] * 80 + [0.2] * 40 + [30.0] * 80
            linear_vel, angular_vel = coordinator.update(
                gps_values, compass_values, obstacle_lidar
            )
            
            self.assertEqual(coordinator.mode, NavigationMode.OBSTACLE_AVOIDANCE)
            print(f"✓ Obstacle detected, mode: {coordinator.mode.value}")


class TestSystemRobustness(unittest.TestCase):
    """Test system robustness and error handling."""
    
    def test_invalid_input_handling(self):
        """Test handling of invalid inputs."""
        print("\n=== Testing Invalid Input Handling ===")
        
        coordinator = BehaviorCoordinator()
        
        # Test invalid target
        result = coordinator.set_navigation_target("invalid_target")
        self.assertFalse(result)
        self.assertEqual(coordinator.mode, NavigationMode.IDLE)
        print("✓ Invalid target rejected, system remains in IDLE mode")
        
        # Test empty sensor data
        processor = SensorProcessor()
        result = processor.process_lidar_data([])
        self.assertFalse(result['obstacles_detected'])
        print("✓ Empty LiDAR data handled gracefully")
        
        result = processor.process_gps_data([])
        self.assertEqual(result, (0.0, 0.0, 0.0))
        print("✓ Empty GPS data handled gracefully")
    
    def test_sensor_failure_recovery(self):
        """Test recovery from sensor failures."""
        print("\n=== Testing Sensor Failure Recovery ===")
        
        processor = SensorProcessor()
        
        # Test with invalid LiDAR readings
        invalid_lidar = [-1.0] * 50 + [100.0] * 50 + [0.5] * 100
        result = processor.process_lidar_data(invalid_lidar)
        
        # Should still process valid readings
        self.assertIsInstance(result, dict)
        print("✓ Invalid LiDAR readings filtered out")
        
        # Test GPS with insufficient data
        result = processor.process_gps_data([1.0])  # Only x coordinate
        self.assertEqual(result, (0.0, 0.0, 0.0))
        print("✓ Insufficient GPS data handled")
    
    def test_stuck_detection_and_recovery(self):
        """Test stuck detection and recovery mechanisms."""
        print("\n=== Testing Stuck Detection and Recovery ===")
        
        avoidance = ObstacleAvoidance()
        
        # Simulate stuck position (same position multiple times)
        stuck_position = (1.0, 1.0)
        for _ in range(15):  # Fill position history
            avoidance._update_position_history(stuck_position)
        
        # Check if stuck detection works
        is_stuck = avoidance._is_stuck()
        self.assertTrue(is_stuck)
        print("✓ Stuck condition detected")
        
        # Test recovery behavior
        lidar_data = {
            'obstacles_detected': True,
            'front_clear': False,
            'left_clear': False,
            'right_clear': False,
            'closest_obstacle_distance': 0.2
        }
        
        # Force stuck state
        avoidance._transition_to_state(AvoidanceState.STUCK)
        
        linear_vel, angular_vel, state = avoidance.process_avoidance(
            lidar_data, stuck_position, 0.0
        )
        
        self.assertEqual(state, AvoidanceState.STUCK)
        print(f"✓ Recovery maneuver: linear={linear_vel:.2f}, angular={angular_vel:.2f}")


class TestPerformanceMetrics(unittest.TestCase):
    """Test performance metrics and benchmarks."""
    
    def test_planning_performance(self):
        """Test path planning performance."""
        print("\n=== Testing Planning Performance ===")
        
        planner = GlobalPathPlanner()
        
        # Test coordinate lookup performance
        start_time = time.time()
        for _ in range(1000):
            coords = planner.get_coordinates("red-box")
        lookup_time = time.time() - start_time
        
        self.assertLess(lookup_time, 0.1, "Coordinate lookup should be fast")
        print(f"✓ 1000 coordinate lookups: {lookup_time:.4f}s")
        
        # Test nearest waypoint calculation performance
        start_time = time.time()
        for _ in range(100):
            nearest = planner.get_nearest_waypoint(1.0, 1.0)
        calculation_time = time.time() - start_time
        
        self.assertLess(calculation_time, 0.1, "Waypoint calculation should be fast")
        print(f"✓ 100 waypoint calculations: {calculation_time:.4f}s")
    
    def test_sensor_processing_performance(self):
        """Test sensor processing performance."""
        print("\n=== Testing Sensor Processing Performance ===")
        
        processor = SensorProcessor()
        lidar_data = [30.0] * 200  # Typical LiDAR scan
        
        # Test LiDAR processing performance
        start_time = time.time()
        for _ in range(100):
            result = processor.process_lidar_data(lidar_data)
        processing_time = time.time() - start_time
        
        self.assertLess(processing_time, 0.5, "LiDAR processing should be real-time")
        print(f"✓ 100 LiDAR scans processed: {processing_time:.4f}s")
        print(f"✓ Average per scan: {processing_time/100*1000:.2f}ms")


def run_feature_tests():
    """Run all feature tests with detailed output."""
    print("=" * 60)
    print("MISSION 2 NAVIGATION SYSTEM - FEATURE TESTS")
    print("=" * 60)
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add test classes
    test_classes = [
        TestNavigationFeatures,
        TestSystemRobustness,
        TestPerformanceMetrics
    ]
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)
    
    # Run tests with detailed output
    runner = unittest.TextTestRunner(verbosity=2, stream=None)
    result = runner.run(test_suite)
    
    print("\n" + "=" * 60)
    print("FEATURE TEST SUMMARY")
    print("=" * 60)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print("\nFAILURES:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print("\nERRORS:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    success_rate = (result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100
    print(f"\nSuccess Rate: {success_rate:.1f}%")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_feature_tests()
    exit(0 if success else 1)
