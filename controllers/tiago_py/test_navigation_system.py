"""
Unit Tests for Mission 2 Navigation System

Comprehensive test suite for validating all components of the TiaGo robot
navigation system including global planning, obstacle avoidance, and task management.

Author: Mission 2 Navigation System
"""

import unittest
import math
import time
from unittest.mock import Mock, patch, MagicMock

# Import system components
from global_planner import GlobalPathPlanner
from sensor_processor import SensorProcessor
from obstacle_avoidance import ObstacleAvoidance, AvoidanceState
from behavior_coordinator import BehaviorCoordinator, NavigationMode
from task_manager import TaskManager, TaskPriority, TaskStatus
from navigation_controller import NavigationController


class TestGlobalPathPlanner(unittest.TestCase):
    """Test cases for PDDL-based global path planning."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.planner = GlobalPathPlanner()
    
    def test_coordinate_mapping(self):
        """Test coordinate mapping for navigation waypoints."""
        # Test known coordinates
        red_coords = self.planner.get_coordinates("red-box")
        self.assertEqual(red_coords, (3.685, 3.01))
        
        green_coords = self.planner.get_coordinates("green-box")
        self.assertEqual(green_coords, (3.685, -3.94))
        
        ducks_coords = self.planner.get_coordinates("ducks-container")
        self.assertEqual(ducks_coords, (-3.07, 3.64))
        
        balls_coords = self.planner.get_coordinates("balls-container")
        self.assertEqual(balls_coords, (-2.795, -4.17))
    
    def test_target_location_mapping(self):
        """Test target to location mapping."""
        self.assertEqual(self.planner.target_locations["red"], "red-box")
        self.assertEqual(self.planner.target_locations["green"], "green-box")
        self.assertEqual(self.planner.target_locations["ducks"], "ducks-container")
        self.assertEqual(self.planner.target_locations["balls"], "balls-container")
    
    def test_nearest_waypoint_calculation(self):
        """Test nearest waypoint calculation."""
        # Test position near start
        nearest = self.planner.get_nearest_waypoint(-1.9, -1.9)
        self.assertEqual(nearest, "start-pos")
        
        # Test position near red box
        nearest = self.planner.get_nearest_waypoint(3.7, 3.0)
        self.assertEqual(nearest, "red-box")
    
    @patch('subprocess.run')
    def test_path_planning_success(self, mock_subprocess):
        """Test successful path planning."""
        # Mock successful pyperplan execution
        mock_subprocess.return_value.returncode = 0
        
        # Mock solution file
        with patch('os.path.exists', return_value=True):
            with patch('builtins.open', mock_open_solution_file()):
                result = self.planner.plan_path("red")
                self.assertIsNotNone(result)
    
    def test_update_current_location(self):
        """Test current location updates."""
        self.planner.update_current_location("corridor-center")
        self.assertEqual(self.planner.current_location, "corridor-center")


class TestSensorProcessor(unittest.TestCase):
    """Test cases for sensor data processing."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.processor = SensorProcessor()
    
    def test_lidar_processing_no_obstacles(self):
        """Test LiDAR processing with no obstacles."""
        # Create clear LiDAR data (all readings at max range)
        lidar_data = [30.0] * 200
        result = self.processor.process_lidar_data(lidar_data)
        
        self.assertFalse(result['obstacles_detected'])
        self.assertTrue(result['front_clear'])
        self.assertTrue(result['left_clear'])
        self.assertTrue(result['right_clear'])
        self.assertEqual(result['closest_obstacle_distance'], float('inf'))
    
    def test_lidar_processing_with_obstacles(self):
        """Test LiDAR processing with obstacles detected."""
        # Create LiDAR data with obstacle in front
        lidar_data = [30.0] * 200
        lidar_data[100] = 0.3  # Obstacle directly in front
        
        result = self.processor.process_lidar_data(lidar_data)
        
        self.assertTrue(result['obstacles_detected'])
        self.assertFalse(result['front_clear'])
        self.assertLess(result['closest_obstacle_distance'], 0.4)
    
    def test_gps_processing(self):
        """Test GPS data processing and filtering."""
        # Test basic GPS processing
        gps_values = [1.0, 2.0, 0.1]
        result = self.processor.process_gps_data(gps_values)
        self.assertEqual(result, (1.0, 2.0, 0.1))
        
        # Test filtering with multiple readings
        gps_values2 = [1.1, 2.1, 0.1]
        result2 = self.processor.process_gps_data(gps_values2)
        # Should be averaged
        self.assertAlmostEqual(result2[0], 1.05, places=2)
        self.assertAlmostEqual(result2[1], 2.05, places=2)
    
    def test_compass_processing(self):
        """Test compass data processing."""
        # Test compass processing
        compass_values = [1.0, 0.0, 0.0]  # Pointing east
        heading = self.processor.process_compass_data(compass_values)
        self.assertAlmostEqual(heading, 0.0, places=2)
        
        compass_values = [0.0, 1.0, 0.0]  # Pointing north
        heading = self.processor.process_compass_data(compass_values)
        self.assertAlmostEqual(heading, math.pi/2, places=2)
    
    def test_safe_direction_calculation(self):
        """Test safe direction calculation for obstacle avoidance."""
        # Test with obstacle on left
        lidar_data = {
            'obstacles_detected': True,
            'front_clear': True,
            'left_clear': False,
            'right_clear': True,
            'closest_obstacle_angle': math.pi/4,
            'left_min_distance': 0.3,
            'right_min_distance': 2.0
        }
        
        safe_direction = self.processor.calculate_safe_direction(lidar_data)
        self.assertLess(safe_direction, 0)  # Should turn right


class TestObstacleAvoidance(unittest.TestCase):
    """Test cases for reactive obstacle avoidance."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.avoidance = ObstacleAvoidance()
    
    def test_initial_state(self):
        """Test initial avoidance state."""
        self.assertEqual(self.avoidance.state, AvoidanceState.NAVIGATING)
    
    def test_target_memory_update(self):
        """Test target memory functionality."""
        target_pos = (3.0, 4.0)
        current_pos = (0.0, 0.0)
        current_heading = 0.0
        
        self.avoidance.update_target_memory(target_pos, current_pos, current_heading)
        
        self.assertEqual(self.avoidance.target_position, target_pos)
        self.assertAlmostEqual(self.avoidance.target_distance, 5.0, places=1)
    
    def test_avoidance_state_transitions(self):
        """Test state transitions in obstacle avoidance."""
        # Test transition to avoiding state
        lidar_data = {
            'obstacles_detected': True,
            'front_clear': False,
            'left_clear': True,
            'right_clear': False,
            'closest_obstacle_distance': 0.3
        }
        
        current_pos = (0.0, 0.0)
        current_heading = 0.0
        
        linear_vel, angular_vel, state = self.avoidance.process_avoidance(
            lidar_data, current_pos, current_heading
        )
        
        self.assertEqual(state, AvoidanceState.AVOIDING)
        self.assertNotEqual(angular_vel, 0.0)  # Should be turning
    
    def test_reset_target_memory(self):
        """Test target memory reset."""
        self.avoidance.target_position = (1.0, 1.0)
        self.avoidance.reset_target_memory()
        
        self.assertIsNone(self.avoidance.target_position)
        self.assertEqual(self.avoidance.state, AvoidanceState.NAVIGATING)


class TestTaskManager(unittest.TestCase):
    """Test cases for task queue management."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_keyboard = Mock()
        self.task_manager = TaskManager(self.mock_keyboard)
    
    def test_add_valid_task(self):
        """Test adding valid navigation tasks."""
        result = self.task_manager.add_task("red", TaskPriority.NORMAL)
        self.assertTrue(result)
        self.assertEqual(len(self.task_manager.task_queue), 1)
    
    def test_add_invalid_task(self):
        """Test adding invalid navigation tasks."""
        result = self.task_manager.add_task("invalid", TaskPriority.NORMAL)
        self.assertFalse(result)
        self.assertEqual(len(self.task_manager.task_queue), 0)
    
    def test_task_priority_ordering(self):
        """Test task priority ordering in queue."""
        self.task_manager.add_task("red", TaskPriority.LOW)
        self.task_manager.add_task("green", TaskPriority.HIGH)
        self.task_manager.add_task("ducks", TaskPriority.NORMAL)
        
        # High priority task should be first
        first_task = self.task_manager.task_queue[0]
        self.assertEqual(first_task.target, "green")
        self.assertEqual(first_task.priority, TaskPriority.HIGH)
    
    def test_task_completion(self):
        """Test task completion workflow."""
        self.task_manager.add_task("red", TaskPriority.NORMAL)
        target = self.task_manager.get_next_task_target()
        
        self.assertEqual(target, "red")
        self.assertIsNotNone(self.task_manager.current_task)
        
        self.task_manager.complete_current_task()
        self.assertEqual(len(self.task_manager.completed_tasks), 1)
    
    def test_queue_management(self):
        """Test queue management operations."""
        self.task_manager.add_task("red", TaskPriority.NORMAL)
        self.task_manager.add_task("green", TaskPriority.NORMAL)
        
        self.assertEqual(len(self.task_manager.task_queue), 2)
        
        self.task_manager.clear_queue()
        self.assertEqual(len(self.task_manager.task_queue), 0)


class TestBehaviorCoordinator(unittest.TestCase):
    """Test cases for behavior coordination."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.coordinator = BehaviorCoordinator()
    
    def test_initial_state(self):
        """Test initial coordinator state."""
        self.assertEqual(self.coordinator.mode, NavigationMode.IDLE)
    
    def test_set_navigation_target(self):
        """Test setting navigation targets."""
        result = self.coordinator.set_navigation_target("red")
        self.assertTrue(result)
        self.assertEqual(self.coordinator.current_target, "red")
        self.assertEqual(self.coordinator.mode, NavigationMode.WAYPOINT_NAVIGATION)
    
    def test_invalid_navigation_target(self):
        """Test setting invalid navigation targets."""
        result = self.coordinator.set_navigation_target("invalid")
        self.assertFalse(result)
        self.assertEqual(self.coordinator.mode, NavigationMode.IDLE)
    
    @patch('global_planner.GlobalPathPlanner.plan_path')
    def test_navigation_update(self, mock_plan_path):
        """Test navigation update loop."""
        # Mock successful path planning
        mock_plan_path.return_value = ["corridor-center", "right-upper", "red-box"]
        
        # Set target
        self.coordinator.set_navigation_target("red")
        
        # Mock sensor data
        gps_values = [0.0, 0.0, 0.1]
        compass_values = [1.0, 0.0, 0.0]
        lidar_values = [30.0] * 200  # No obstacles
        
        # Update navigation
        linear_vel, angular_vel = self.coordinator.update(
            gps_values, compass_values, lidar_values
        )
        
        # Should produce some control output
        self.assertIsInstance(linear_vel, float)
        self.assertIsInstance(angular_vel, float)


def mock_open_solution_file():
    """Mock solution file for testing."""
    solution_content = """(move tiago start-pos corridor-center)
(navigate-to-room tiago corridor-center right-room right-upper)
(approach-target tiago right-upper red-box)"""
    
    return unittest.mock.mock_open(read_data=solution_content)


class TestIntegration(unittest.TestCase):
    """Integration tests for the complete navigation system."""
    
    def setUp(self):
        """Set up integration test fixtures."""
        self.mock_robot = Mock()
        self.mock_keyboard = Mock()
        
        # Mock robot devices
        self.mock_robot.getDevice.return_value = Mock()
        self.mock_robot.getBasicTimeStep.return_value = 32
        
    def test_system_initialization(self):
        """Test complete system initialization."""
        # This would test the main system initialization
        # In a real scenario, we'd mock the Robot class
        pass
    
    def test_end_to_end_navigation(self):
        """Test end-to-end navigation scenario."""
        # This would test a complete navigation scenario
        # from task input to goal completion
        pass


if __name__ == '__main__':
    # Run all tests
    unittest.main(verbosity=2)
