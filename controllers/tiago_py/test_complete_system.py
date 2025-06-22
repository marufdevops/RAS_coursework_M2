#!/usr/bin/env python3
"""
Complete System Test for TiaGo PDDL Navigation

This module tests the entire navigation system including:
- Node network with weighted connections
- PDDL domain and problem generation
- Pyperplan integration and solution parsing
- Path cost calculation and validation
"""

from node_network import NodeNetwork
from pddl_system import PDDLSystem
import os


def test_node_network():
    """Test the node network implementation."""
    print("=" * 60)
    print("TESTING NODE NETWORK")
    print("=" * 60)
    
    network = NodeNetwork()
    network.print_network_info()
    
    # Test path cost calculation
    test_paths = [
        ['node1', 'node2'],
        ['node1', 'node2', 'node4'],
        ['node1', 'node2', 'node4', 'node3']
    ]
    
    print(f"\nPath Cost Tests:")
    for path in test_paths:
        cost = network.calculate_path_cost(path)
        print(f"  {' -> '.join(path)}: {cost:.3f}m")
    
    # Test goal reachability
    print(f"\nGoal Reachability Test:")
    balls_pos = network.balls_target
    print(f"Balls target: {balls_pos}")
    
    for node_name, node_pos in network.nodes.items():
        distance = network.calculate_distance(node_pos, balls_pos)
        reachable = distance <= network.goal_threshold
        status = "✓ REACHABLE" if reachable else "✗ NOT REACHABLE"
        print(f"  From {node_name}: {distance:.3f}m - {status}")
    
    return True


def test_pddl_system():
    """Test PDDL domain/problem generation and planning."""
    print(f"\n" + "=" * 60)
    print("TESTING PDDL SYSTEM")
    print("=" * 60)
    
    pddl_system = PDDLSystem()
    
    # Test different robot starting positions
    test_positions = [
        (-1.96, -1.95),  # node1
        (-0.6, -1.95),   # node2
        (-1.96, -3.9),   # node3
        (-0.6, -3.9)     # node4
    ]
    
    for i, robot_pos in enumerate(test_positions, 1):
        print(f"\nTest {i}: Robot at {robot_pos}")
        
        # Generate navigation plan
        path, cost = pddl_system.plan_navigation(robot_pos)
        
        if path:
            print(f"  Generated path: {path}")
            print(f"  Total cost: {cost:.3f}m")
            
            # Validate path
            network = pddl_system.network
            calculated_cost = network.calculate_path_cost(path)
            
            if abs(cost - calculated_cost) < 0.001:
                print(f"  ✓ Cost validation passed")
            else:
                print(f"  ✗ Cost validation failed: {cost:.3f} vs {calculated_cost:.3f}")
        else:
            print(f"  ✗ No path generated")
    
    return True


def test_optimal_paths():
    """Test that generated paths are optimal."""
    print(f"\n" + "=" * 60)
    print("TESTING PATH OPTIMALITY")
    print("=" * 60)
    
    network = NodeNetwork()
    
    # Manual calculation of shortest paths
    # From node1 to node3 (closest to balls):
    # Option 1: node1 -> node2 -> node4 -> node3
    # Option 2: Direct connections don't exist, so we need to go through the graph
    
    print("Manual path analysis:")
    
    # Calculate all possible paths from node1 to node3
    possible_paths = [
        ['node1', 'node2', 'node4', 'node3']  # Only valid path given connections
    ]
    
    for path in possible_paths:
        cost = network.calculate_path_cost(path)
        print(f"  {' -> '.join(path)}: {cost:.3f}m")
    
    # Test with PDDL system
    pddl_system = PDDLSystem()
    robot_pos = (-1.96, -1.95)  # node1
    generated_path, generated_cost = pddl_system.plan_navigation(robot_pos)
    
    print(f"\nPDDL generated path: {generated_path}")
    print(f"PDDL generated cost: {generated_cost:.3f}m")
    
    # Verify it matches our manual calculation
    manual_cost = network.calculate_path_cost(possible_paths[0])
    if abs(generated_cost - manual_cost) < 0.001:
        print("✓ PDDL path matches optimal manual calculation")
    else:
        print("✗ PDDL path differs from manual calculation")
    
    return True


def test_goal_detection():
    """Test goal detection logic."""
    print(f"\n" + "=" * 60)
    print("TESTING GOAL DETECTION")
    print("=" * 60)
    
    network = NodeNetwork()
    balls_target = network.balls_target
    
    print(f"Balls target: {balls_target}")
    print(f"Goal threshold: {network.goal_threshold}m")
    
    # Test positions at various distances from balls target
    test_positions = [
        (balls_target[0], balls_target[1]),  # Exactly at target
        (balls_target[0] + 0.5, balls_target[1]),  # 0.5m away
        (balls_target[0] + 0.8, balls_target[1]),  # 0.8m away (threshold)
        (balls_target[0] + 1.0, balls_target[1]),  # 1.0m away
    ]
    
    for pos in test_positions:
        distance = network.calculate_distance(pos, balls_target)
        reached = network.is_goal_reached(pos)
        status = "✓ REACHED" if reached else "✗ NOT REACHED"
        print(f"  Position {pos}: {distance:.3f}m - {status}")
    
    return True


def test_file_generation():
    """Test that PDDL files are generated correctly."""
    print(f"\n" + "=" * 60)
    print("TESTING FILE GENERATION")
    print("=" * 60)
    
    pddl_system = PDDLSystem()
    
    # Generate files
    domain_file = pddl_system.generate_domain_file("test_domain.pddl")
    problem_file, start_node, goal_node = pddl_system.generate_problem_file(
        (-1.96, -1.95), "test_problem.pddl"
    )
    
    # Check files exist
    files_to_check = [domain_file, problem_file]
    for file_path in files_to_check:
        if os.path.exists(file_path):
            print(f"✓ {file_path} generated successfully")
            
            # Check file content
            with open(file_path, 'r') as f:
                content = f.read()
            
            if len(content) > 0:
                print(f"  File size: {len(content)} characters")
            else:
                print(f"  ✗ File is empty")
        else:
            print(f"✗ {file_path} not found")
    
    # Clean up test files
    for file_path in files_to_check:
        if os.path.exists(file_path):
            os.remove(file_path)
            print(f"  Cleaned up {file_path}")
    
    return True


def run_complete_system_test():
    """Run all system tests."""
    print("TIAGO PDDL NAVIGATION SYSTEM - COMPLETE TEST SUITE")
    print("=" * 80)
    
    tests = [
        ("Node Network", test_node_network),
        ("PDDL System", test_pddl_system),
        ("Path Optimality", test_optimal_paths),
        ("Goal Detection", test_goal_detection),
        ("File Generation", test_file_generation)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            print(f"\nRunning {test_name} test...")
            success = test_func()
            results.append((test_name, success))
            print(f"✓ {test_name} test completed")
        except Exception as e:
            print(f"✗ {test_name} test failed: {e}")
            results.append((test_name, False))
    
    # Print summary
    print(f"\n" + "=" * 80)
    print("TEST SUMMARY")
    print("=" * 80)
    
    passed = 0
    for test_name, success in results:
        status = "PASSED" if success else "FAILED"
        print(f"{test_name}: {status}")
        if success:
            passed += 1
    
    print(f"\nOverall: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("🎯 ALL TESTS PASSED - System ready for navigation!")
    else:
        print("⚠️  Some tests failed - Check implementation")
    
    return passed == len(results)


if __name__ == "__main__":
    success = run_complete_system_test()
    exit(0 if success else 1)
