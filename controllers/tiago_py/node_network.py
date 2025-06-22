"""
Node Network System for TiaGo Robot Navigation

This module defines a weighted graph-based node network for PDDL navigation
focusing on the "balls" target with optimal path planning.
Based on the new graph structure from the provided screenshot.
"""

import math


class NodeNetwork:
    def __init__(self):
        """Initialize the node network with coordinates and weighted connections."""
        
        # Define node coordinates based on the updated graph from screenshot
        self.nodes = {
            'node1': (-1.96, -1.45),   # Top-left node
            'node2': (-0.47, -1.95),   # Top-center node
            'node3': (-0.6, -3.9),     # Bottom-right node
            'node4': (-1.75, -3.9),    # Bottom-left node (CORRECTED coordinates)
            'node5': (-0.06, -3.38),   # Middle-right node
            'node6': (2.64, -3.61)     # Far-right node (near green goal)
        }

        # Define bidirectional connections based on the updated graph structure
        self.connections = [
            # Original connections
            ('node1', 'node2'),  # Top horizontal connection
            ('node2', 'node3'),  # Left vertical connection
            ('node3', 'node4'),  # Bottom horizontal connection
            # New connections with added nodes
            ('node2', 'node5'),  # Connection from node2 to new node5
            ('node5', 'node6'),  # Connection from node5 to node6
            ('node5', 'node4'),  # Cross connection: node5 to node4
            ('node4', 'node6')   # Long diagonal connection: node4 to node6
        ]
        
        # Calculate weights (Euclidean distances) for all connections
        self.weights = {}
        self._calculate_connection_weights()
        
        # Goal threshold for reaching targets
        self.goal_threshold = 0.8
        
        # Target coordinates (from goalchecker.py)
        self.balls_target = self._calculate_balls_center()
        self.green_target = self._calculate_green_center()
        
    def _calculate_balls_center(self):
        """Calculate center coordinates of balls target from goalchecker polygon."""
        # From goalchecker.py: balls polygon coordinates
        balls_polygon = [(-2.55, -3.67), (-2.25, -4.20), (-3.09, -4.67), (-3.34, -4.15)]

        # Calculate center point
        x_coords = [point[0] for point in balls_polygon]
        y_coords = [point[1] for point in balls_polygon]
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)

        return (center_x, center_y)

    def _calculate_green_center(self):
        """Calculate center coordinates of green target from goalchecker polygon."""
        # From goalchecker.py: green polygon coordinates
        green_polygon = [(3.18, -4.86), (4.19, -4.86), (4.19, -3.02), (3.18, -3.02)]

        # Calculate center point
        x_coords = [point[0] for point in green_polygon]
        y_coords = [point[1] for point in green_polygon]
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)

        return (center_x, center_y)
    
    def _calculate_connection_weights(self):
        """Calculate Euclidean distance weights for all connections."""
        for node1, node2 in self.connections:
            pos1 = self.nodes[node1]
            pos2 = self.nodes[node2]
            distance = self.calculate_distance(pos1, pos2)
            
            # Store bidirectional weights
            self.weights[(node1, node2)] = distance
            self.weights[(node2, node1)] = distance
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions."""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def get_connection_weight(self, node1, node2):
        """Get the weight (distance) of a connection between two nodes."""
        return self.weights.get((node1, node2), float('inf'))
    
    def get_connected_nodes(self, node):
        """Get all nodes connected to the given node."""
        connected = []
        for node1, node2 in self.connections:
            if node1 == node:
                connected.append(node2)
            elif node2 == node:
                connected.append(node1)
        return connected
    
    def find_closest_node(self, position):
        """Find the closest node to a given position."""
        min_distance = float('inf')
        closest_node = None
        
        for node_name, node_pos in self.nodes.items():
            distance = self.calculate_distance(position, node_pos)
            if distance < min_distance:
                min_distance = distance
                closest_node = node_name
        
        return closest_node, min_distance
    
    def is_goal_reached(self, robot_position, target='balls'):
        """Check if robot has reached the specified target within threshold."""
        if target == 'balls':
            target_pos = self.balls_target
        elif target == 'green':
            target_pos = self.green_target
        else:
            return False

        distance = self.calculate_distance(robot_position, target_pos)
        return distance <= self.goal_threshold
    
    def get_node_coordinates(self, node_name):
        """Get coordinates of a specific node."""
        return self.nodes.get(node_name)

    def get_target_coordinates(self, target_name):
        """Get coordinates of a specific target goal."""
        if target_name == 'balls':
            return self.balls_target
        elif target_name == 'green':
            return self.green_target
        else:
            return None
    
    def get_all_connections(self):
        """Get all connections with their weights."""
        return [(node1, node2, self.weights[(node1, node2)]) 
                for node1, node2 in self.connections]
    
    def calculate_path_cost(self, path):
        """Calculate total cost (distance) of a path through nodes."""
        if len(path) < 2:
            return 0.0
        
        total_cost = 0.0
        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]
            cost = self.get_connection_weight(current_node, next_node)
            if cost == float('inf'):
                return float('inf')  # Invalid path
            total_cost += cost
        
        return total_cost
    
    def calculate_angle_to_target(self, current_pos, target_pos):
        """Calculate angle from current position to target position."""
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        return math.atan2(dy, dx)
    
    def print_network_info(self):
        """Print detailed network information for debugging."""
        print("Node Network Information:")
        print("=" * 50)
        
        print(f"\nNodes ({len(self.nodes)}):")
        for node_name, coords in self.nodes.items():
            print(f"  {node_name}: {coords}")
        
        print(f"\nTargets:")
        print(f"  Balls Target: {self.balls_target}")
        print(f"  Green Target: {self.green_target}")
        print(f"Goal Threshold: {self.goal_threshold}m")
        
        print(f"\nConnections ({len(self.connections)}):")
        for node1, node2 in self.connections:
            weight = self.weights[(node1, node2)]
            print(f"  {node1} <-> {node2}: {weight:.3f}m")
        
        print(f"\nClosest node to balls target:")
        closest_node, distance = self.find_closest_node(self.balls_target)
        print(f"  {closest_node}: {distance:.3f}m")


if __name__ == "__main__":
    # Test the node network
    network = NodeNetwork()
    network.print_network_info()
    
    # Test path cost calculation
    test_path = ['node1', 'node2', 'node4']
    cost = network.calculate_path_cost(test_path)
    print(f"\nTest path {test_path} cost: {cost:.3f}m")
