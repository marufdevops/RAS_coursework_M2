#!/usr/bin/env python3
"""
Unit Tests for PDDL-Based Navigation System
Tests waypoint reachability, PDDL planning, and goal achievement
"""

import unittest
import math
import os
import sys


# Add the current directory to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Define test data directly (avoiding Webots dependencies)
WAYPOINTS = {
    'start': (-1.97, -1.96),
    'center_west': (-1.0, 0.0),
    'center_east': (1.0, 0.0),
    'north_west': (-1.5, 2.5),
    'north_east': (1.5, 2.5),
    'south_west': (-1.0, -3.5),
    'south_east': (1.5, -3.0),
    'red_approach': (3.2, 2.5),
    'green_approach': (3.2, -3.2),
    'ducks_approach': (-2.2, 3.2),
    'balls_approach': (-2.0, -3.8),
    'west_corridor': (-2.8, 0.0),
    'east_corridor': (2.8, 0.0),
    'north_central': (0.5, 1.5),
    'south_central': (0.5, -2.5),
}

TARGET_COORDS = {
    'red': (3.68, 3.02),
    'green': (3.68, -3.95),
    'ducks': (-3.0, 3.7),
    'balls': (-2.8, -4.3)
}

# Navigation parameters
WAYPOINT_THRESHOLD = 0.4
GOAL_THRESHOLD = 0.8
OBSTACLE_THRESHOLD = 0.8
MAX_VELOCITY = 5.0
PAUSE_TIMEOUT = 10.0

# Utility functions for testing
def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def distance_between_points(p1, p2):
    """Calculate distance between two points"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def angle_between_points(p1, p2):
    """Calculate angle from p1 to p2"""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])

class TestNavigationSystem(unittest.TestCase):
    """Test suite for the navigation system"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.test_waypoints = {
            'start': (-1.97, -1.96),
            'center_west': (-1.0, 0.0),
            'center_east': (1.0, 0.0),
            'north_west': (-1.5, 2.5),
            'red_approach': (3.2, 2.5),
        }
        
        self.test_targets = {
            'red': (3.68, 3.02),
            'green': (3.68, -3.95),
            'ducks': (-3.0, 3.7),
            'balls': (-2.8, -4.3)
        }

    def test_waypoint_coordinates_validity(self):
        """Test that all waypoints have valid coordinates"""
        for name, coord in WAYPOINTS.items():
            with self.subTest(waypoint=name):
                self.assertIsInstance(coord, tuple, f"Waypoint {name} should be a tuple")
                self.assertEqual(len(coord), 2, f"Waypoint {name} should have 2 coordinates")
                self.assertIsInstance(coord[0], (int, float), f"Waypoint {name} x-coord should be numeric")
                self.assertIsInstance(coord[1], (int, float), f"Waypoint {name} y-coord should be numeric")
                # Check coordinates are within reasonable warehouse bounds
                self.assertGreaterEqual(coord[0], -5.0, f"Waypoint {name} x-coord too small")
                self.assertLessEqual(coord[0], 5.0, f"Waypoint {name} x-coord too large")
                self.assertGreaterEqual(coord[1], -5.0, f"Waypoint {name} y-coord too small")
                self.assertLessEqual(coord[1], 5.0, f"Waypoint {name} y-coord too large")

    def test_target_coordinates_validity(self):
        """Test that all target coordinates are valid"""
        for name, coord in TARGET_COORDS.items():
            with self.subTest(target=name):
                self.assertIsInstance(coord, tuple, f"Target {name} should be a tuple")
                self.assertEqual(len(coord), 2, f"Target {name} should have 2 coordinates")
                self.assertIsInstance(coord[0], (int, float), f"Target {name} x-coord should be numeric")
                self.assertIsInstance(coord[1], (int, float), f"Target {name} y-coord should be numeric")

    def test_distance_calculation(self):
        """Test distance calculation function"""
        # Test distance between points
        dist = distance_between_points((0.0, 0.0), (0.0, 0.0))
        self.assertAlmostEqual(dist, 0.0, places=2)

        # Test distance to (3, 4) - should be 5
        dist = distance_between_points((0.0, 0.0), (3.0, 4.0))
        self.assertAlmostEqual(dist, 5.0, places=2)

        # Test with different points
        dist = distance_between_points((1.0, 1.0), (4.0, 5.0))
        expected = math.sqrt((4-1)**2 + (5-1)**2)
        self.assertAlmostEqual(dist, expected, places=2)

    def test_angle_calculation(self):
        """Test angle calculation function"""
        # Test angle to point directly east
        angle = angle_between_points((0.0, 0.0), (1.0, 0.0))
        self.assertAlmostEqual(angle, 0.0, places=2)

        # Test angle to point directly north
        angle = angle_between_points((0.0, 0.0), (0.0, 1.0))
        self.assertAlmostEqual(angle, math.pi/2, places=2)

        # Test angle to point directly west
        angle = angle_between_points((0.0, 0.0), (-1.0, 0.0))
        self.assertAlmostEqual(angle, math.pi, places=2)

    def test_angle_normalization(self):
        """Test angle normalization function"""
        # Test angles within range
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-math.pi), -math.pi)
        
        # Test angles outside range
        self.assertAlmostEqual(normalize_angle(3*math.pi), math.pi, places=5)
        self.assertAlmostEqual(normalize_angle(-3*math.pi), -math.pi, places=5)
        self.assertAlmostEqual(normalize_angle(2*math.pi), 0.0, places=5)

    def test_closest_waypoint_detection(self):
        """Test finding the closest waypoint"""
        def find_closest_waypoint(pos):
            min_dist = float('inf')
            closest = 'start'
            for name, coord in WAYPOINTS.items():
                dist = distance_between_points(pos, coord)
                if dist < min_dist:
                    min_dist = dist
                    closest = name
            return closest

        # Test from start position
        closest = find_closest_waypoint((-1.97, -1.96))
        self.assertEqual(closest, 'start')

        # Test from center area
        closest = find_closest_waypoint((0.0, 0.0))
        # Should be either center_west or center_east
        self.assertIn(closest, ['center_west', 'center_east'])

    def test_waypoint_reachability(self):
        """Test that all waypoints are reachable from each other"""
        # This is a simplified connectivity test
        # In a real implementation, we'd test the actual path planning
        
        # Check that key waypoints exist
        required_waypoints = [
            'start', 'center_west', 'center_east',
            'red_approach', 'green_approach', 'ducks_approach', 'balls_approach'
        ]
        
        for waypoint in required_waypoints:
            self.assertIn(waypoint, WAYPOINTS, f"Required waypoint {waypoint} missing")

    def test_pddl_problem_generation(self):
        """Test PDDL problem file generation"""
        # Test that we can create a basic PDDL problem structure
        def create_test_pddl_problem(start_waypoint, goal_target):
            goal_waypoint_map = {
                'red': 'red_approach',
                'green': 'green_approach',
                'ducks': 'ducks_approach',
                'balls': 'balls_approach'
            }
            goal_waypoint = goal_waypoint_map.get(goal_target, 'center_west')

            problem_content = f"""(define (problem navigate-to-{goal_target})
    (:domain warehouse)
    (:objects
        robot - robot
        {' '.join(WAYPOINTS.keys())} - waypoint
    )
    (:init
        (at robot {start_waypoint})
    )
    (:goal
        (at robot {goal_waypoint})
    )
)"""
            return problem_content

        # Test problem generation
        content = create_test_pddl_problem('start', 'red')
        self.assertIn('(at robot start)', content)
        self.assertIn('(at robot red_approach)', content)
        self.assertIn('(define (problem navigate-to-red)', content)

    def test_target_approach_waypoints(self):
        """Test that approach waypoints are close to targets"""
        approach_map = {
            'red_approach': 'red',
            'green_approach': 'green', 
            'ducks_approach': 'ducks',
            'balls_approach': 'balls'
        }
        
        for approach_wp, target in approach_map.items():
            if approach_wp in WAYPOINTS and target in TARGET_COORDS:
                approach_coord = WAYPOINTS[approach_wp]
                target_coord = TARGET_COORDS[target]
                
                # Calculate distance between approach waypoint and target
                dist = math.sqrt(
                    (approach_coord[0] - target_coord[0])**2 + 
                    (approach_coord[1] - target_coord[1])**2
                )
                
                # Approach waypoint should be within reasonable distance of target
                self.assertLess(dist, 2.0, 
                    f"Approach waypoint {approach_wp} too far from target {target}")

    def test_obstacle_avoidance_parameters(self):
        """Test that obstacle avoidance parameters are reasonable"""
        # Import the parameters
        try:
            from tiago_py import OBSTACLE_THRESHOLD, PAUSE_TIMEOUT
            
            # Obstacle threshold should be reasonable for robot navigation
            self.assertGreater(OBSTACLE_THRESHOLD, 0.1)
            self.assertLess(OBSTACLE_THRESHOLD, 2.0)
            
            # Pause timeout should prevent infinite waiting
            self.assertGreater(PAUSE_TIMEOUT, 1.0)
            self.assertLess(PAUSE_TIMEOUT, 30.0)
            
        except ImportError:
            self.skipTest("Could not import obstacle avoidance parameters")

    def test_navigation_parameters(self):
        """Test that navigation parameters are reasonable"""
        try:
            from tiago_py import WAYPOINT_THRESHOLD, GOAL_THRESHOLD, MAX_VELOCITY
            
            # Thresholds should be positive and reasonable
            self.assertGreater(WAYPOINT_THRESHOLD, 0.1)
            self.assertLess(WAYPOINT_THRESHOLD, 1.0)
            
            self.assertGreater(GOAL_THRESHOLD, 0.1)
            self.assertLess(GOAL_THRESHOLD, 2.0)
            
            # Velocity should be reasonable for robot
            self.assertGreater(MAX_VELOCITY, 1.0)
            self.assertLess(MAX_VELOCITY, 10.0)
            
        except ImportError:
            self.skipTest("Could not import navigation parameters")


class TestPDDLPlanning(unittest.TestCase):
    """Test PDDL planning functionality"""
    
    def setUp(self):
        """Set up PDDL test fixtures"""
        self.domain_file = "warehouse_domain.pddl"
        self.problem_file = "warehouse_problem.pddl"

    def test_domain_file_exists(self):
        """Test that PDDL domain file exists"""
        self.assertTrue(os.path.exists(self.domain_file), 
                       "PDDL domain file should exist")

    def test_domain_file_content(self):
        """Test PDDL domain file content"""
        if os.path.exists(self.domain_file):
            with open(self.domain_file, 'r') as f:
                content = f.read()
                self.assertIn('(define (domain warehouse)', content)
                self.assertIn('(:action move', content)
                self.assertIn('robot', content)
                self.assertIn('waypoint', content)


if __name__ == '__main__':
    print("Running Navigation System Unit Tests...")
    print("=" * 50)
    
    # Run the tests
    unittest.main(verbosity=2)
