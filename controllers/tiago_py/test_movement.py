#!/usr/bin/env python3
"""
Test Movement Logic

This script tests the movement and rotation calculations without requiring Webots.
"""

import math
from node_network import NodeNetwork


def test_angle_calculations():
    """Test angle calculations for robot movement."""
    print("=== Testing Angle Calculations ===")
    
    network = NodeNetwork()
    
    # Test cases: robot position -> target position
    test_cases = [
        ((-1.96, -1.95), (-0.6, -1.95)),   # node1 to node2 (horizontal right)
        ((-0.6, -1.95), (-0.6, -3.9)),     # node2 to node3 (vertical down)
        ((-0.6, -3.9), (-1.96, -3.9)),     # node3 to node4 (horizontal left)
    ]
    
    for i, (robot_pos, target_pos) in enumerate(test_cases, 1):
        angle = network.calculate_angle_to_target(robot_pos, target_pos)
        angle_degrees = math.degrees(angle)
        distance = network.calculate_distance(robot_pos, target_pos)
        
        print(f"Test {i}:")
        print(f"  Robot: {robot_pos} -> Target: {target_pos}")
        print(f"  Angle: {angle:.3f} rad ({angle_degrees:.1f}°)")
        print(f"  Distance: {distance:.3f}m")
        
        # Describe direction
        if -0.1 < angle < 0.1:
            direction = "East (right)"
        elif 1.47 < angle < 1.67:
            direction = "North (up)"
        elif -1.67 < angle < -1.47:
            direction = "South (down)"
        elif abs(angle) > 3.0:
            direction = "West (left)"
        else:
            direction = f"Custom ({angle_degrees:.1f}°)"
        
        print(f"  Direction: {direction}")
        print()


def test_path_validation():
    """Test that the generated path is valid."""
    print("=== Testing Path Validation ===")
    
    network = NodeNetwork()
    
    # Test the expected path from PDDL
    expected_path = ['node1', 'node2', 'node3', 'node4']
    
    print(f"Testing path: {' -> '.join(expected_path)}")
    
    # Validate each connection
    valid_path = True
    total_cost = 0.0
    
    for i in range(len(expected_path) - 1):
        current_node = expected_path[i]
        next_node = expected_path[i + 1]
        
        # Check if connection exists
        connection_exists = (current_node, next_node) in [(n1, n2) for n1, n2 in network.connections] or \
                           (next_node, current_node) in [(n1, n2) for n1, n2 in network.connections]
        
        if connection_exists:
            cost = network.get_connection_weight(current_node, next_node)
            total_cost += cost
            print(f"  ✓ {current_node} -> {next_node}: {cost:.3f}m")
        else:
            print(f"  ✗ {current_node} -> {next_node}: NO CONNECTION")
            valid_path = False
    
    print(f"\nPath validation: {'✓ VALID' if valid_path else '✗ INVALID'}")
    print(f"Total path cost: {total_cost:.3f}m")
    
    # Compare with network calculation
    network_cost = network.calculate_path_cost(expected_path)
    print(f"Network calculated cost: {network_cost:.3f}m")
    
    if abs(total_cost - network_cost) < 0.001:
        print("✓ Cost calculations match")
    else:
        print("✗ Cost calculations differ")


def test_goal_proximity():
    """Test proximity to balls target from each node."""
    print("=== Testing Goal Proximity ===")
    
    network = NodeNetwork()
    balls_target = network.balls_target
    
    print(f"Balls target: {balls_target}")
    print(f"Goal threshold: {network.goal_threshold}m")
    
    closest_node = None
    min_distance = float('inf')
    
    for node_name, node_pos in network.nodes.items():
        distance = network.calculate_distance(node_pos, balls_target)
        reachable = distance <= network.goal_threshold
        status = "✓ REACHABLE" if reachable else "✗ NOT REACHABLE"
        
        print(f"  {node_name}: {distance:.3f}m - {status}")
        
        if distance < min_distance:
            min_distance = distance
            closest_node = node_name
    
    print(f"\nClosest node to balls: {closest_node} ({min_distance:.3f}m)")
    
    # Check if any node can reach the goal
    if min_distance <= network.goal_threshold:
        print("✓ Goal is reachable from at least one node")
    else:
        print("⚠️  Goal is not reachable from any node - robot will need to move closer")


if __name__ == "__main__":
    print("MOVEMENT LOGIC TEST SUITE")
    print("=" * 50)
    
    test_angle_calculations()
    test_path_validation()
    test_goal_proximity()
    
    print("=" * 50)
    print("Test complete. If all tests pass, movement logic should work correctly.")
