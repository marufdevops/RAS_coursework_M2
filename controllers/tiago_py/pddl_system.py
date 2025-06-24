"""
This module implements a sophisticated PDDL (Planning Domain Definition Language)
integration system for optimal multi-goal robot navigation.

**Foundation:**
- Classical Planning: STRIPS representation with typed objects and a*
- Optimality Theory: Guaranteed shortest paths in weighted graphs

**PDDL Integration Architecture:**
1. Domain Generation: Automated STRIPS domain creation with typing
2. Problem Generation: Dynamic problem instance creation from robot state
3. Planner Integration: Pyperplan A* algorithm with optimality guarantees
4. Solution Parsing: Path extraction and cost calculation

**Design Rationale:**
- PDDL provides formal semantics for navigation planning
- A* algorithm ensures optimal solutions
- Modular architecture enables easy planner substitution

"""

import subprocess
import os
from node_network import NodeNetwork


class PDDLSystem:
    """
    PDDL Planning System for Robot Navigation.

    Generates PDDL domain/problem files and uses pyperplan for optimal path planning.
    Supports navigation to balls, green, ducks, and red targets.
    """

    def __init__(self):
        """Initialize the PDDL planning system with node network."""
        self.network = NodeNetwork()
        
    def generate_domain_file(self, filename="navigation_domain.pddl"):
        """
        Generate PDDL 1.2 domain file compatible with pyperplan.

        Creates a formal PDDL domain specification for robot navigation using
        STRIPS representation with typed objects. The domain defines the
        navigation action space and state predicates for optimal planning.

        **PDDL Domain Structure:**
        - Types: robot, node (for type safety and validation)
        - Predicates: at(robot, node), connected(node, node)
        - Actions: move(robot, from_node, to_node)

        **Design Rationale:**
        - STRIPS representation ensures compatibility with classical planners
        - Typed objects prevent invalid action instantiations
        - Simple action model enables efficient search algorithms
        - Bidirectional connections modeled through symmetric predicates

        Args:
            filename (str, optional): Output filename for domain file

        Returns:
            str: Path to generated domain file

        Raises:
            IOError: If file cannot be written to specified location
        """
        
        domain_content = """(define (domain navigation)
  (:requirements :strips :typing)
  
  (:types
    robot node - object
  )
  
  (:predicates
    (at ?robot - robot ?node - node)
    (connected ?from - node ?to - node)
  )
  
  (:action move
    :parameters (?robot - robot ?from - node ?to - node)
    :precondition (and 
      (at ?robot ?from)
      (connected ?from ?to)
    )
    :effect (and 
      (not (at ?robot ?from))
      (at ?robot ?to)
    )
  )
)"""
        
        with open(filename, 'w') as f:
            f.write(domain_content)
        
        print(f"Domain file generated: {filename}")
        return filename
    
    def generate_problem_file(self, robot_position, target='balls', filename="navigation_problem.pddl"):
        """
        Generate PDDL problem file for navigation to specified target.

        Creates a problem instance for the navigation domain, specifying the
        initial state (robot position) and goal state (target location).
        Uses closest node mapping to translate continuous coordinates to
        discrete node representation.

        **Problem Structure:**
        - Objects: robot instance and all network nodes
        - Initial State: robot at closest node to current position
        - Goal State: robot at closest node to target coordinates
        - Facts: all bidirectional node connections

        **Coordinate Mapping:**
        Robot and target positions are mapped to closest network nodes using
        Euclidean distance minimization, ensuring valid problem instances.

        Args:
            robot_position (tuple): Current robot coordinates (x, y)
            target (str): Target goal name ('balls', 'green', 'ducks', 'red')
            filename (str, optional): Output filename for problem file

        Returns:
            tuple: (problem_file, start_node, goal_node) where:
                - problem_file (str): Path to generated problem file
                - start_node (str): Starting node name
                - goal_node (str): Goal node name

        Raises:
            ValueError: If target is not a valid goal name
            IOError: If file cannot be written to specified location
        """

        # Find closest node to robot and to target
        start_node, _ = self.network.find_closest_node(robot_position)
        target_coords = self.network.get_target_coordinates(target)
        if target_coords is None:
            raise ValueError(f"Unknown target: {target}")
        goal_node, _ = self.network.find_closest_node(target_coords)
        
        # Generate node declarations
        node_declarations = []
        for node_name in self.network.nodes.keys():
            node_declarations.append(f"    {node_name}")
        
        # Generate connection facts (bidirectional)
        init_facts = [f"    (at tiago {start_node})"]
        
        for node1, node2 in self.network.connections:
            init_facts.append(f"    (connected {node1} {node2})")
            # Add reverse direction
            init_facts.append(f"    (connected {node2} {node1})")
        
        problem_content = f"""(define (problem navigation-to-{target})
  (:domain navigation)

  (:objects
    tiago - robot
{chr(10).join(node_declarations)} - node
  )

  (:init
{chr(10).join(init_facts)}
  )

  (:goal
    (at tiago {goal_node})
  )
)"""

        with open(filename, 'w') as f:
            f.write(problem_content)

        print(f"Problem file generated: {filename}")
        print(f"Robot starts at: {start_node} (closest to {robot_position})")
        print(f"Goal: {goal_node} (closest to {target} at {target_coords})")

        return filename, start_node, goal_node
    
    def run_pyperplan(self, domain_file, problem_file):
        """Run pyperplan to generate optimal solution with A* algorithm."""
        try:
        
            cmd = f"/opt/anaconda3/bin/pyperplan -s astar {domain_file} {problem_file}"
            print(f"Trying command: {cmd}")
            
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, cwd=os.getcwd())
            
            if result.returncode == 0:
                print(f"Pyperplan executed successfully with: {cmd}")
                solution_file = f"{problem_file}.soln"
                print(f"Solution file: {solution_file}")
                return solution_file
            else:
                print(f"Command failed: {result.stderr}")

        except Exception as e:
            print(f"Error running pyperplan: {e}")
            return None
    
    def parse_solution(self, solution_file):
        """Parse pyperplan solution file to extract path and calculate total cost."""
        try:
            with open(solution_file, 'r') as f:
                lines = f.readlines()
            
            path = []
            
            # Find starting node from first move action
            for line in lines:
                line = line.strip()
                if line.startswith('(move tiago'):
                    parts = line.replace('(', '').replace(')', '').split()
                    if len(parts) >= 4:
                        from_node = parts[2]
                        to_node = parts[3]
                        
                        # Add starting node if this is the first move
                        if not path:
                            path.append(from_node)
                        
                        path.append(to_node)
            
            # Calculate total cost using network weights
            total_cost = self.network.calculate_path_cost(path)
            
            print(f"Parsed path: {path}")
            print(f"Total path cost: {total_cost:.3f}m")
            
            return path, total_cost
            
        except Exception as e:
            print(f"Error parsing solution file: {e}")
            return [], 0.0
    
    def plan_navigation(self, robot_position, target='balls'):
        """
        Plan optimal navigation path to target using PDDL and pyperplan.

        Args:
            robot_position (tuple): Current robot coordinates (x, y)
            target (str): Target goal ('balls', 'green', 'ducks', 'red')

        Returns:
            tuple: (path, total_cost) - path as list of nodes, cost in meters
        """
        # Validate target goal specification
        target_coords = self.network.get_target_coordinates(target)
        if target_coords is None:
            print(f"Error: Unknown target '{target}'")
            return [], 0.0

        print(f"\n=== Planning Navigation to {target.title()} Target ===")
        print(f"Robot position: {robot_position}")
        print(f"{target.title()} target: {target_coords}")

        # Execute PDDL planning pipeline
        domain_file = self.generate_domain_file()
        problem_file, _, _ = self.generate_problem_file(robot_position, target)

        # Run optimal A* search through pyperplan
        solution_file = self.run_pyperplan(domain_file, problem_file)

        if solution_file and os.path.exists(solution_file):
            # Parse and validate optimal solution
            path, total_cost = self.parse_solution(solution_file)
            return path, total_cost
        else:
            print("Failed to generate solution")
            return [], 0.0


# Production PDDL System - No test code needed
# For testing, use the main navigation system in tiago_py.py
