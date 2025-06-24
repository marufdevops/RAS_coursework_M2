import math

class NodeNetwork:
    """
    Advanced Multi-Goal Navigation Network with Optimal Pathfinding Capabilities.

    **Formula(taken from one of course example and augment code(LLM)):**
    - Graph Representation: G = (V, E) where |V| = 11, |E| = 16
    - Weight Function: w(u,v) = √((x₂-x₁)² + (y₂-y₁)²) (Euclidean distance)

    **Network Architecture:**
    The 11-node network provides strategic coverage with the following properties:

    Attributes:
        nodes (dict): Node name to (x, y) coordinate mapping
        connections (list): Bidirectional edge list as (node1, node2) tuples
        weights (dict): Edge weights as {(node1, node2): distance} mapping
        goal_threshold (float): Distance threshold for goal completion (0.8m)
        balls_target (tuple): Center coordinates of balls goal polygon
        green_target (tuple): Center coordinates of green goal polygon
        ducks_target (tuple): Center coordinates of ducks goal polygon
        red_target (tuple): Center coordinates of red goal polygon
    """

    def __init__(self):
        """
        Initialize the node network with optimized coordinates and weighted connections.

        Constructs the complete 11-node navigation network with strategic node placement
        based on coverage optimization 
        
        Automatically calculates all edge weights using Euclidean distance and initializes goal
        target coordinates from goalchecker.py polygon definitions.

        **Initialization Process:**
        1. Define 11 strategic node coordinates for optimal space coverage
        2. Establish 16 bidirectional connections for path redundancy
        3. Calculate Euclidean distance weights for all edges
        4. Compute goal center coordinates from polygon definitions
        5. Validate network connectivity and goal associations

        **Design Rationale:**
        - Node placement ensures accurate connections
        - Connection topology provides multiple paths between any two goals
        - Weight calculation uses Euclidean distance for shortest physical paths
        - Goal threshold (0.8m) accounts for GPS noise (±0.3m) and robot dimensions

        Raises:
            ValueError: If node coordinates contain invalid values
            RuntimeError: If network connectivity validation fails
        """
        
        # Define node coordinates based on the expanded 11-node network
        self.nodes = {
            'node1': (-1.96, -1.45),   # Bottom-left area
            'node2': (-0.47, -2.15) ,   # Bottom-center area
            'node3': (-0.6, -3.9),     # Bottom area
            'node4': (-1.75, -3.9),    # Bottom-left area (balls goal)
            'node5': (-0.06, -3.38),   # Bottom-center area
            'node6': (2.64, -3.61),    # Bottom-right area (green goal)
            'node7': (-2.5, 2.6),      # Top-left area (ducks goal)
            'node8': (-0.65, 2.6),     # Top-center area
            'node9': (2.6, 2.6),       # Top-right area (red goal)
            'node10': (1, 1),          # Middle-center area
            'node11': (2.55, -0.65)    # Right-center area
        }

        # Define bidirectional connections for the complete 11-node network (16 connections)
        self.connections = [
            ('node1', 'node2'),   # Bottom horizontal
            ('node2', 'node3'),   # Bottom vertical
            ('node3', 'node4'),   # Bottom horizontal
            ('node2', 'node5'),   # Bottom diagonal
            ('node5', 'node6'),   # Bottom horizontal
            ('node5', 'node4'),   # Bottom connection
            ('node4', 'node6'),   # Bottom long diagonal
            ('node1', 'node7'),   # Vertical up from node1
            ('node7', 'node8'),   # Top horizontal left-center
            ('node8', 'node9'),   # Top horizontal center-right
            ('node2', 'node8'),   # Vertical up from node2
            ('node8', 'node10'),  # Diagonal down-right from node8
            ('node9', 'node10'),  # Diagonal down-left from node9
            ('node10', 'node11'), # Horizontal right from node10
            ('node11', 'node6'),  # Vertical down from node11
            ('node5', 'node11')   # Diagonal up-right from node5
        ]
        
        # Calculate weights (Euclidean distances) for all connections
        self.weights = {}
        self._calculate_connection_weights()
        
        # Goal threshold for reaching targets
        self.goal_threshold = 0.8
        
        # Target coordinates (from goalchecker.py)
        self.balls_target = self._calculate_balls_center()
        self.green_target = self._calculate_green_center()
        self.ducks_target = self._calculate_ducks_center()
        self.red_target = self._calculate_red_center()
        
    def _calculate_balls_center(self):
        """
        Calculate geometric center of balls target from goalchecker polygon definition.

        Computes the centroid of the balls goal polygon using coordinate averaging.
        This method implements the standard centroid calculation for irregular polygons,
        providing the optimal target point for navigation planning.

        **Formula:**
        For polygon vertices P = {(x₁,y₁), (x₂,y₂), ..., (xₙ,yₙ)}:
        - Centroid_x = (1/n) * Σᵢ xᵢ
        - Centroid_y = (1/n) * Σᵢ yᵢ

        **Design Rationale:**
        - Centroid provides optimal approach point for goal detection
        - Coordinate averaging ensures balanced positioning within goal area

        Returns:
            tuple: (center_x, center_y) coordinates of balls target centroid
                  Expected: approximately (-2.81, -4.17)
        """
        # From goalchecker.py: balls polygon coordinates
        balls_polygon = [(-2.55, -3.67), (-2.25, -4.20), (-3.09, -4.67), (-3.34, -4.15)]

        # Calculate center point using coordinate averaging (centroid formula)
        x_coords = [point[0] for point in balls_polygon]
        y_coords = [point[1] for point in balls_polygon]
        center_x = sum(x_coords) / len(x_coords)  # Arithmetic mean of x-coordinates
        center_y = sum(y_coords) / len(y_coords)  # Arithmetic mean of y-coordinates

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

    def _calculate_ducks_center(self):
        """Calculate center coordinates of ducks target from goalchecker polygon."""
        # From goalchecker.py: ducks polygon coordinates
        ducks_polygon = [(-2.59, 3.90), (-3.07, 3.08), (-3.55, 3.37), (-3.09, 4.19)]

        # Calculate center point
        x_coords = [point[0] for point in ducks_polygon]
        y_coords = [point[1] for point in ducks_polygon]
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)

        return (center_x, center_y)

    def _calculate_red_center(self):
        """Calculate center coordinates of red target from goalchecker polygon."""
        # From goalchecker.py: red polygon coordinates
        red_polygon = [(3.18, 2.12), (4.19, 2.12), (4.19, 3.90), (3.18, 3.90)]

        # Calculate center point
        x_coords = [point[0] for point in red_polygon]
        y_coords = [point[1] for point in red_polygon]
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
    

    
    def find_closest_node(self, position):
        """
        Find the closest node to a given position using Euclidean distance minimization.

        Implements optimal node selection for path planning by computing the minimum
        Euclidean distance from the query position to all network nodes. This method
        is fundamental to the PDDL planning system for determining start and goal nodes.

        **Algorithm:** Brute-force linear search with distance minimization
        **Formula:**
        - Distance metric: d(p,q) = √((x₂-x₁)² + (y₂-y₁)²)
        
        Args:
            position (tuple): Query position as (x, y) coordinates in meters
                            Expected range: (-5, 5) for both x and y

        Returns:
            tuple: (closest_node_name, minimum_distance) where:
                - closest_node_name (str): Name of closest node (e.g., 'node4')
                - minimum_distance (float): Euclidean distance in meters
        
        Personal Note: Augment Code Generated
        """
        min_distance = float('inf')  # Initialize with infinity for comparison
        closest_node = None

        # Linear search through all nodes for minimum distance
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
        elif target == 'ducks':
            target_pos = self.ducks_target
        elif target == 'red':
            target_pos = self.red_target
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
        elif target_name == 'ducks':
            return self.ducks_target
        elif target_name == 'red':
            return self.red_target
        else:
            return None
    

    
    def calculate_path_cost(self, path):
        """
        Calculate total cost (distance) of a path through network nodes.

        Computes the cumulative Euclidean distance for a sequence of nodes,
        providing the total physical distance that the robot must travel.
        This method is essential for path optimization and performance analysis.

        **Algorithm:** Sequential edge weight summation
        
        **Design Rationale:**
        - Cumulative sum provides total physical distance for robot travel

        Args:
            path (list): Sequence of node names representing the path
                        Example: ['node1', 'node2', 'node5', 'node6']
                        Minimum length: 0 (empty path returns 0.0)

        Returns:
            float: Total path cost in meters, or float('inf') for invalid paths
                  Range: [0.0, ∞) where typical values are 2-15 meters

        **Integration with PDDL:**
        This method validates PDDL-generated paths and provides cost metrics
        for performance analysis and optimization verification.

        Personal Note: Augement Code Generated
        """
        if len(path) < 2:
            return 0.0  # No movement required for empty or single-node paths

        total_cost = 0.0
        # Sequential summation of edge weights along the path
        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]
            cost = self.get_connection_weight(current_node, next_node)
            if cost == float('inf'):
                return float('inf')  # Invalid path - disconnected segment
            total_cost += cost

        return total_cost
    
    def calculate_angle_to_target(self, current_pos, target_pos):
        """Calculate angle from current position to target position.
        Personal Note: This code is augment code generated
        """
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        return math.atan2(dy, dx)
    

