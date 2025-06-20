#!/usr/bin/env python3
"""
Generate all PDDL solutions for navigation planning
This script pre-generates solutions for all start->goal combinations
"""

import os
import subprocess
import json

# Waypoint definitions (same as in tiago_py.py)
WAYPOINTS = {
    # Core navigation waypoints (safe open areas)
    'start': (-1.97, -1.96),    # Robot start position
    'center_west': (-1.0, 0.0), # West side of central wall
    'center_east': (1.0, 0.0),  # East side of central wall
    'north_west': (-1.5, 2.5),  # Northwest corridor (clear of cones)
    'north_east': (1.5, 2.5),   # Northeast corridor (clear of cones)
    'south_west': (-1.0, -3.5), # Southwest corridor (clear of cones)
    'south_east': (1.5, -3.0),  # Southeast corridor (clear of cones)

    # Target approach waypoints (safe distances from obstacles)
    'red_approach': (3.2, 2.5),    # Approach to red box (clear of traffic cones)
    'green_approach': (3.2, -3.2), # Approach to green box (clear of traffic cones)
    'ducks_approach': (-2.2, 3.2), # Approach to ducks (clear of boundary)
    'balls_approach': (-2.0, -3.8), # Approach to balls (clear of traffic cones)

    # Intermediate waypoints for complex navigation
    'west_corridor': (-2.8, 0.0),  # Western corridor
    'east_corridor': (2.8, 0.0),   # Eastern corridor
    'north_central': (0.5, 1.5),   # North of central wall
    'south_central': (0.5, -2.5),  # South of central wall
}

def find_pyperplan_path():
    """Find the absolute path to pyperplan executable"""
    # Common paths where pyperplan might be installed
    possible_paths = [
        "/opt/anaconda3/bin/pyperplan",
        "/usr/local/bin/pyperplan", 
        "/usr/bin/pyperplan",
        "pyperplan"  # fallback to PATH
    ]
    
    for path in possible_paths:
        try:
            # Test if pyperplan works at this path
            result = subprocess.run([path, "--help"], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print(f"Found pyperplan at: {path}")
                return path
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
            continue
    
    # If not found, try to find it using which command
    try:
        result = subprocess.run(["which", "pyperplan"], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            path = result.stdout.strip()
            print(f"Found pyperplan using which: {path}")
            return path
    except:
        pass
    
    print("ERROR: pyperplan not found in any common locations")
    return None

def write_pddl_problem(start_waypoint, goal_target):
    """Generate PDDL problem file for navigation"""
    # Determine goal waypoint based on target
    goal_waypoint_map = {
        'red': 'red_approach',
        'green': 'green_approach',
        'ducks': 'ducks_approach',
        'balls': 'balls_approach'
    }
    goal_waypoint = goal_waypoint_map.get(goal_target, 'center')

    problem_content = f"""(define (problem navigate-to-{goal_target})
    (:domain warehouse)
    (:objects
        robot - robot
        {' '.join(WAYPOINTS.keys())} - waypoint
    )
    (:init
        (at robot {start_waypoint})
        ; Waypoint connections (bidirectional) - based on actual warehouse layout
        ; Start connections
        (connected start center_west)
        (connected center_west start)
        (connected start south_west)
        (connected south_west start)

        ; Central area connections (around the central wall)
        (connected center_west center_east)
        (connected center_east center_west)
        (connected center_west north_west)
        (connected north_west center_west)
        (connected center_west south_west)
        (connected south_west center_west)
        (connected center_east north_east)
        (connected north_east center_east)
        (connected center_east south_east)
        (connected south_east center_east)

        ; North-south corridor connections
        (connected north_west north_central)
        (connected north_central north_west)
        (connected north_east north_central)
        (connected north_central north_east)
        (connected south_west south_central)
        (connected south_central south_west)
        (connected south_east south_central)
        (connected south_central south_east)

        ; East-west corridor connections
        (connected center_west west_corridor)
        (connected west_corridor center_west)
        (connected center_east east_corridor)
        (connected east_corridor center_east)

        ; Target approach connections
        (connected north_east red_approach)
        (connected red_approach north_east)
        (connected east_corridor red_approach)
        (connected red_approach east_corridor)

        (connected south_east green_approach)
        (connected green_approach south_east)
        (connected east_corridor green_approach)
        (connected green_approach east_corridor)

        (connected north_west ducks_approach)
        (connected ducks_approach north_west)
        (connected west_corridor ducks_approach)
        (connected ducks_approach west_corridor)

        (connected south_west balls_approach)
        (connected balls_approach south_west)
        (connected west_corridor balls_approach)
        (connected balls_approach west_corridor)
    )
    (:goal
        (at robot {goal_waypoint})
    )
)"""

    with open("warehouse_problem.pddl", 'w') as f:
        f.write(problem_content)

def generate_all_solutions():
    """Generate all PDDL solutions and save to reference file"""
    print("=== Generating All PDDL Solutions ===")
    
    # Find pyperplan
    pyperplan_path = find_pyperplan_path()
    if not pyperplan_path:
        print("Cannot proceed without pyperplan")
        return False
    
    # Common starting waypoints
    start_waypoints = ['start', 'center_west', 'center_east', 'north_west', 'north_east', 'south_west', 'south_east']
    targets = ['red', 'green', 'ducks', 'balls']
    
    solutions = {}
    total_combinations = len(start_waypoints) * len(targets)
    current = 0
    
    for target in targets:
        solutions[target] = {}
        for start in start_waypoints:
            current += 1
            print(f"\n[{current}/{total_combinations}] Generating: {start} -> {target}")
            
            try:
                # Generate problem file
                write_pddl_problem(start, target)
                
                # Run pyperplan with absolute path
                domain_file = os.path.abspath("warehouse_domain.pddl")
                problem_file = os.path.abspath("warehouse_problem.pddl")
                cmd = [pyperplan_path, "-H", "hff", "-s", "astar", domain_file, problem_file]
                
                # Set up environment
                env = os.environ.copy()
                env['PATH'] = '/opt/anaconda3/bin:' + env.get('PATH', '')
                env['PYTHONPATH'] = '/opt/anaconda3/lib/python3.12/site-packages:' + env.get('PYTHONPATH', '')
                
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=30, env=env)
                
                if result.returncode == 0:
                    solution_file = problem_file + '.soln'
                    if os.path.exists(solution_file):
                        with open(solution_file, 'r') as f:
                            solution_content = f.read().strip()
                        
                        # Parse solution to extract waypoints
                        waypoints = []
                        lines = solution_content.split('\n')
                        for line in lines:
                            line = line.strip()
                            if line.startswith('(move robot'):
                                parts = line.split()
                                if len(parts) >= 4:
                                    waypoint = parts[3].rstrip(')')
                                    waypoints.append(waypoint)
                        
                        solutions[target][start] = waypoints
                        os.remove(solution_file)  # Clean up
                        print(f"✓ Solution: {waypoints}")
                    else:
                        print(f"✗ No solution file generated")
                        solutions[target][start] = []
                else:
                    print(f"✗ Planning failed (return code {result.returncode})")
                    print(f"Error: {result.stderr}")
                    solutions[target][start] = []
                    
            except subprocess.TimeoutExpired:
                print(f"✗ Timeout")
                solutions[target][start] = []
            except Exception as e:
                print(f"✗ Error: {e}")
                solutions[target][start] = []
    
    # Save solutions to reference file
    with open("pddl_solutions_reference.json", 'w') as f:
        json.dump(solutions, f, indent=2)
    
    print(f"\n=== Generation Complete ===")
    print(f"Reference solutions saved to: pddl_solutions_reference.json")
    
    # Print summary
    total_solutions = sum(len([s for s in solutions[target].values() if s]) for target in targets)
    print(f"Successfully generated {total_solutions}/{total_combinations} solutions")
    
    return True

def main():
    """Main function"""
    if not os.path.exists("warehouse_domain.pddl"):
        print("ERROR: warehouse_domain.pddl not found")
        print("Please run this script from the controllers/tiago_py directory")
        return
    
    success = generate_all_solutions()
    
    if success:
        print("\n✅ All PDDL solutions generated successfully!")
        print("The navigation system can now use these pre-generated solutions.")
    else:
        print("\n❌ Failed to generate solutions")

if __name__ == "__main__":
    main()
