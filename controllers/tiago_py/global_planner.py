"""
Global Path Planning System using PDDL and pyperplan

This module implements strategic navigation planning for the TiaGo robot
in the warehouse environment using PDDL-based deliberative planning.

Author: Mission 2 Navigation System
"""

import os
import tempfile
import subprocess
import re
from typing import List, Tuple, Optional, Dict
import math


class GlobalPathPlanner:
    """
    PDDL-based global path planner for warehouse navigation.
    
    Provides strategic route planning between warehouse locations using
    pyperplan for optimal path generation with obstacle awareness.
    """
    
    def __init__(self, domain_file: str = "warehouse_domain.pddl"):
        """
        Initialize the global path planner.
        
        Args:
            domain_file: Path to PDDL domain file
        """
        self.domain_file = domain_file
        self.current_location = "start-pos"
        
        # Coordinate mapping for navigation waypoints
        self.location_coordinates = {
            "start-pos": (-1.97, -1.96),
            "corridor-center": (0.0, 0.0),
            "corridor-north": (0.0, 2.0),
            "corridor-south": (0.0, -2.0),
            "left-upper": (-3.0, 3.5),
            "left-lower": (-3.0, -4.0),
            "right-upper": (3.7, 3.0),
            "right-lower": (3.7, -4.0),
            "red-box": (3.685, 3.01),      # Center of red box area
            "green-box": (3.685, -3.94),   # Center of green box area
            "ducks-container": (-3.07, 3.64),  # Center of ducks area
            "balls-container": (-2.795, -4.17)  # Center of balls area
        }
        
        # Target to location mapping
        self.target_locations = {
            "red": "red-box",
            "green": "green-box", 
            "ducks": "ducks-container",
            "balls": "balls-container"
        }
    
    def create_problem_file(self, target: str, current_pos: str = None) -> str:
        """
        Create a PDDL problem file for navigation to target.
        
        Args:
            target: Target location name (red, green, ducks, balls)
            current_pos: Current robot position (defaults to self.current_location)
            
        Returns:
            Path to generated problem file
        """
        if current_pos is None:
            current_pos = self.current_location
            
        target_location = self.target_locations.get(target, target)
        
        problem_content = f"""(define (problem warehouse-navigation-task)
  (:domain warehouse-navigation)
  
  (:objects
    ;; Robot
    tiago - robot
    
    ;; Rooms
    left-room - room
    right-room - room
    center-corridor - room
    
    ;; Navigation waypoints
    start-pos - location
    left-upper - location
    left-lower - location
    right-upper - location
    right-lower - location
    corridor-center - location
    corridor-north - location
    corridor-south - location
    
    ;; Target locations
    red-box - target
    green-box - target
    ducks-container - target
    balls-container - target
  )
  
  (:init
    ;; Initial robot position
    (robot-at tiago {current_pos})
    
    ;; Room membership
    (in-room left-upper left-room)
    (in-room left-lower left-room)
    (in-room right-upper right-room)
    (in-room right-lower right-room)
    (in-room corridor-center center-corridor)
    (in-room corridor-north center-corridor)
    (in-room corridor-south center-corridor)
    
    ;; Target room assignments
    (in-room ducks-container left-room)
    (in-room balls-container left-room)
    (in-room red-box right-room)
    (in-room green-box right-room)
    
    ;; Connectivity - bidirectional paths
    ;; Start position connections
    (connected start-pos corridor-center)
    (connected corridor-center start-pos)
    
    ;; Corridor internal connections
    (connected corridor-center corridor-north)
    (connected corridor-north corridor-center)
    (connected corridor-center corridor-south)
    (connected corridor-south corridor-center)
    
    ;; Left room connections
    (connected corridor-north left-upper)
    (connected left-upper corridor-north)
    (connected corridor-south left-lower)
    (connected left-lower corridor-south)
    (connected left-upper left-lower)
    (connected left-lower left-upper)
    
    ;; Right room connections
    (connected corridor-north right-upper)
    (connected right-upper corridor-north)
    (connected corridor-south right-lower)
    (connected right-lower corridor-south)
    (connected right-upper right-lower)
    (connected right-lower right-upper)
    
    ;; Target accessibility
    (connected left-upper ducks-container)
    (connected ducks-container left-upper)
    (connected left-lower balls-container)
    (connected balls-container left-lower)
    (connected right-upper red-box)
    (connected red-box right-upper)
    (connected right-lower green-box)
    (connected green-box right-lower)
    
    ;; All targets are accessible initially
    (target-accessible red-box)
    (target-accessible green-box)
    (target-accessible ducks-container)
    (target-accessible balls-container)
    
    ;; All paths are clear initially
    (path-clear start-pos corridor-center)
    (path-clear corridor-center start-pos)
    (path-clear corridor-center corridor-north)
    (path-clear corridor-north corridor-center)
    (path-clear corridor-center corridor-south)
    (path-clear corridor-south corridor-center)
    (path-clear corridor-north left-upper)
    (path-clear left-upper corridor-north)
    (path-clear corridor-south left-lower)
    (path-clear left-lower corridor-south)
    (path-clear left-upper left-lower)
    (path-clear left-lower left-upper)
    (path-clear corridor-north right-upper)
    (path-clear right-upper corridor-north)
    (path-clear corridor-south right-lower)
    (path-clear right-lower corridor-south)
    (path-clear right-upper right-lower)
    (path-clear right-lower right-upper)
    (path-clear left-upper ducks-container)
    (path-clear ducks-container left-upper)
    (path-clear left-lower balls-container)
    (path-clear balls-container left-lower)
    (path-clear right-upper red-box)
    (path-clear red-box right-upper)
    (path-clear right-lower green-box)
    (path-clear green-box right-lower)
  )
  
  (:goal (robot-at tiago {target_location}))
)"""
        
        # Create temporary problem file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.pddl', delete=False) as f:
            f.write(problem_content)
            return f.name
    
    def plan_path(self, target: str, current_pos: str = None) -> Optional[List[str]]:
        """
        Generate optimal path to target using PDDL planner with fallback.

        Args:
            target: Target location name
            current_pos: Current robot position

        Returns:
            List of waypoint locations for navigation, or None if no path found
        """
        # First try PDDL planning
        pddl_result = self._try_pddl_planning(target, current_pos)
        if pddl_result is not None:
            return pddl_result

        # Fallback to simple rule-based planning
        print("PDDL planning failed, using fallback rule-based planning")
        return self._fallback_planning(target, current_pos)

    def _try_pddl_planning(self, target: str, current_pos: str = None) -> Optional[List[str]]:
        """Try PDDL-based planning."""
        try:
            # Create problem file
            problem_file = self.create_problem_file(target, current_pos)

            # Run pyperplan with full path
            import shutil
            pyperplan_path = shutil.which('pyperplan')
            if pyperplan_path is None:
                # Try common installation paths
                possible_paths = [
                    '/opt/anaconda3/bin/pyperplan',
                    '/usr/local/bin/pyperplan'
                ]
                for path in possible_paths:
                    if os.path.exists(path):
                        pyperplan_path = path
                        break

                if pyperplan_path is None:
                    # Try python module approach
                    try:
                        result = subprocess.run(['python', '-m', 'pyperplan', self.domain_file, problem_file],
                                              capture_output=True, text=True, cwd=os.path.dirname(__file__), timeout=10)
                        if result.returncode == 0:
                            return self._parse_solution(problem_file)
                    except:
                        pass
                    return None

            cmd = [pyperplan_path, self.domain_file, problem_file]
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(__file__), timeout=10)

            if result.returncode != 0:
                print(f"PDDL planning failed: {result.stderr}")
                return None

            return self._parse_solution(problem_file)

        except Exception as e:
            print(f"PDDL planning error: {e}")
            return None

    def _parse_solution(self, problem_file: str) -> Optional[List[str]]:
        """Parse PDDL solution file."""
        try:
            solution_file = problem_file + '.soln'
            if not os.path.exists(solution_file):
                return None

            with open(solution_file, 'r') as f:
                solution_lines = f.readlines()

            # Extract waypoints from solution
            waypoints = []
            for line in solution_lines:
                line = line.strip()
                if line.startswith('(move') or line.startswith('(navigate-to-room') or line.startswith('(approach-target'):
                    # Extract destination from action
                    parts = line.split()
                    if len(parts) >= 4:
                        destination = parts[-1].rstrip(')')
                        waypoints.append(destination)

            # Clean up temporary files
            os.unlink(problem_file)
            if os.path.exists(solution_file):
                os.unlink(solution_file)

            return waypoints if waypoints else None

        except Exception as e:
            print(f"Solution parsing error: {e}")
            return None

    def _fallback_planning(self, target: str, current_pos: str = None) -> Optional[List[str]]:
        """
        Simple rule-based planning as fallback when PDDL fails.

        Args:
            target: Target location name
            current_pos: Current robot position

        Returns:
            List of waypoint locations for navigation
        """
        if current_pos is None:
            current_pos = self.current_location

        target_location = self.target_locations.get(target, target)

        # Simple rule-based paths for each target
        fallback_paths = {
            "red-box": ["corridor-center", "corridor-north", "right-upper", "red-box"],
            "green-box": ["corridor-center", "corridor-south", "right-lower", "green-box"],
            "ducks-container": ["corridor-center", "corridor-north", "left-upper", "ducks-container"],
            "balls-container": ["corridor-center", "corridor-south", "left-lower", "balls-container"]
        }

        if target_location in fallback_paths:
            path = fallback_paths[target_location]

            # Remove waypoints we've already passed
            if current_pos in path:
                start_index = path.index(current_pos)
                return path[start_index + 1:]  # Return remaining path
            else:
                return path  # Return full path

        print(f"No fallback path available for target: {target}")
        return None
    
    def get_coordinates(self, location: str) -> Tuple[float, float]:
        """
        Get world coordinates for a location.
        
        Args:
            location: Location name
            
        Returns:
            (x, y) coordinates in world frame
        """
        return self.location_coordinates.get(location, (0.0, 0.0))
    
    def update_current_location(self, location: str):
        """
        Update the robot's current location.
        
        Args:
            location: New current location
        """
        self.current_location = location
    
    def get_nearest_waypoint(self, x: float, y: float) -> str:
        """
        Find the nearest navigation waypoint to given coordinates.
        
        Args:
            x, y: Current robot coordinates
            
        Returns:
            Name of nearest waypoint
        """
        min_distance = float('inf')
        nearest_waypoint = "start-pos"
        
        for location, (loc_x, loc_y) in self.location_coordinates.items():
            distance = math.sqrt((x - loc_x)**2 + (y - loc_y)**2)
            if distance < min_distance:
                min_distance = distance
                nearest_waypoint = location
                
        return nearest_waypoint
