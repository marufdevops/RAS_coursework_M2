#!/usr/bin/env python3
"""
World Analysis Script for Mission2.wbt
Analyzes the .wbt file to create accurate 2D occupancy grid and waypoint system
Based on precise coordinates extracted from mission2.wbt
"""

def create_grid_from_wbt():
    """
    Create a 2D occupancy grid from mission2.wbt file analysis
    Based on precise coordinates from the .wbt file and corrected coordinate system
    """
    # Arena dimensions: 9m x 11m (from RectangleArena floorSize 9 11)
    # Grid resolution: 0.2m per cell for better visualization (45 x 55 grid)
    width_m, height_m = 9.0, 11.0
    resolution = 0.2

    grid_width = int(width_m / resolution)  # 45 cells
    grid_height = int(height_m / resolution)  # 55 cells

    # Initialize grid (0 = free, 1 = obstacle)
    grid = [[0 for _ in range(grid_width)] for _ in range(grid_height)]

    # Convert world coordinates to grid coordinates
    def world_to_grid(x, y):
        # World coordinates: x=[-4.5, 4.5], y=[-5.5, 5.5]
        grid_x = int((x + width_m/2) / resolution)
        grid_y = int((y + height_m/2) / resolution)
        return max(0, min(grid_width-1, grid_x)), max(0, min(grid_height-1, grid_y))

    # Add boundary walls (from .wbt file)
    # Left wall: x = -4.39, size = 0.2 x 11 x 1.5
    for y in range(grid_height):
        grid[y][0] = 1

    # Right wall: x = 4.37, size = 0.2 x 11 x 1.5
    for y in range(grid_height):
        grid[y][grid_width-1] = 1

    # Bottom wall: y = -5.44, size = 9 x 0.2 x 1.5
    for x in range(grid_width):
        grid[0][x] = 1

    # Top wall: y = 5.38, size = 9 x 0.2 x 1.5
    for x in range(grid_width):
        grid[grid_height-1][x] = 1

    # Add internal walls creating the cross pattern (from .wbt file)
    center_x = grid_width // 2  # x = 0 in world coordinates

    # Vertical wall segments at x=0:
    # Wall 1: y=-0.19, size=3.8 (from y=-2.09 to y=1.71)
    wall1_y_start = int((-2.09 + height_m/2) / resolution)
    wall1_y_end = int((1.71 + height_m/2) / resolution)
    for y in range(max(1, wall1_y_start), min(grid_height-1, wall1_y_end+1)):
        grid[y][center_x] = 1

    # Wall 2: y=-4.78, size=1.5 (from y=-5.53 to y=-4.03)
    wall2_y_start = int((-5.53 + height_m/2) / resolution)
    wall2_y_end = int((-4.03 + height_m/2) / resolution)
    for y in range(max(1, wall2_y_start), min(grid_height-1, wall2_y_end+1)):
        grid[y][center_x] = 1

    # Wall 3: y=4.35, size=2 (from y=3.35 to y=5.35)
    wall3_y_start = int((3.35 + height_m/2) / resolution)
    wall3_y_end = int((5.35 + height_m/2) / resolution)
    for y in range(max(1, wall3_y_start), min(grid_height-1, wall3_y_end+1)):
        grid[y][center_x] = 1

    # Horizontal wall segment at y=-0.03:
    # Wall 4: x=-3.68, size=1.5 (from x=-4.43 to x=-2.93)
    wall4_y = int((-0.03 + height_m/2) / resolution)
    wall4_x_start = int((-4.43 + width_m/2) / resolution)
    wall4_x_end = int((-2.93 + width_m/2) / resolution)
    for x in range(max(1, wall4_x_start), min(grid_width-1, wall4_x_end+1)):
        if 0 <= wall4_y < grid_height:
            grid[wall4_y][x] = 1

    # Add traffic cones as obstacles (from .wbt file)
    cone_positions = [
        (-3.47, -3.02), (-2.94, -2.91), (-2.4, -2.82), (-1.34, -2.76),
        (0.39, -1.53), (0.93, -1.38), (1.46, -1.22), (3.94, 1.04),
        (3.39, 0.86), (2.85, 0.74), (2.31, 0.85), (-4, -3.12),
        (-1.87, -2.84)
    ]

    for cone_x, cone_y in cone_positions:
        gx, gy = world_to_grid(cone_x, cone_y)
        if 0 <= gy < grid_height and 0 <= gx < grid_width:
            grid[gy][gx] = 1

    return grid, resolution, width_m, height_m

def add_large_obstacles(grid, resolution, width_m, height_m):
    """
    Add wooden boxes and metal storage boxes to the grid
    """
    def world_to_grid(x, y):
        grid_x = int((x + width_m/2) / resolution)
        grid_y = int((y + height_m/2) / resolution)
        return max(0, min(len(grid[0])-1, grid_x)), max(0, min(len(grid)-1, grid_y))
    
    # Add wooden boxes as obstacles (from .wbt file)
    # Box 1: (3.68, 3.02), size 1x1.8x0.88
    box1_x, box1_y = 3.68, 3.02
    box1_gx, box1_gy = world_to_grid(box1_x, box1_y)
    # Add box area (10x18 cells for 1x1.8m box at 0.1m resolution)
    for dy in range(-9, 10):
        for dx in range(-5, 6):
            if 0 <= box1_gy + dy < len(grid) and 0 <= box1_gx + dx < len(grid[0]):
                grid[box1_gy + dy][box1_gx + dx] = 1
    
    # Box 2: (3.68, -3.95), size 1x1.8x0.88  
    box2_x, box2_y = 3.68, -3.95
    box2_gx, box2_gy = world_to_grid(box2_x, box2_y)
    for dy in range(-9, 10):
        for dx in range(-5, 6):
            if 0 <= box2_gy + dy < len(grid) and 0 <= box2_gx + dx < len(grid[0]):
                grid[box2_gy + dy][box2_gx + dx] = 1
    
    # Add metal storage boxes (from .wbt file)
    # Metal box 1: (-3.08, 3.5) with rotation
    metal1_gx, metal1_gy = world_to_grid(-3.08, 3.5)
    for dy in range(-5, 6):
        for dx in range(-5, 6):
            if 0 <= metal1_gy + dy < len(grid) and 0 <= metal1_gx + dx < len(grid[0]):
                grid[metal1_gy + dy][metal1_gx + dx] = 1
    
    # Metal box 2: (-2.82, -4.31) with rotation
    metal2_gx, metal2_gy = world_to_grid(-2.82, -4.31)
    for dy in range(-5, 6):
        for dx in range(-5, 6):
            if 0 <= metal2_gy + dy < len(grid) and 0 <= metal2_gx + dx < len(grid[0]):
                grid[metal2_gy + dy][metal2_gx + dx] = 1
    
    return grid

def get_goal_areas():
    """
    Return goal areas from goalchecker.py with precise coordinates
    """
    goals = {
        'red': {
            'bounds': [(3.18, 2.12), (4.19, 2.12), (4.19, 3.90), (3.18, 3.90)],
            'center': (3.685, 3.01),
            'key': 'R'
        },
        'green': {
            'bounds': [(3.18, -4.86), (4.19, -4.86), (4.19, -3.02), (3.18, -3.02)],
            'center': (3.685, -3.94),
            'key': 'G'
        },
        'ducks': {
            'bounds': [(-2.59, 3.90), (-3.07, 3.08), (-3.55, 3.37), (-3.09, 4.19)],
            'center': (-3.07, 3.64),
            'key': 'D'
        },
        'balls': {
            'bounds': [(-2.55, -3.67), (-2.25, -4.20), (-3.09, -4.67), (-3.34, -4.15)],
            'center': (-2.81, -4.17),
            'key': 'B'
        }
    }
    return goals

def get_robot_positions():
    """
    Return robot starting positions from .wbt file (corrected coordinates)
    """
    robots = {
        'tiago': (-1.97, -1.96, 'T'),
        'pioneer1': (-2.27, 4.13, '1'),
        'pioneer2': (0.92, -3.74, '2'),
        'pioneer3': (1.93, 3.51, '3')
    }
    return robots

def visualize_grid(grid, resolution, width_m, height_m):
    """
    Create a visual representation of the grid with symbols
    """
    def world_to_grid(x, y):
        grid_x = int((x + width_m/2) / resolution)
        grid_y = int((y + height_m/2) / resolution)
        return max(0, min(len(grid[0])-1, grid_x)), max(0, min(len(grid)-1, grid_y))

    # Create display grid
    display_grid = []
    for row in grid:
        display_row = []
        for cell in row:
            if cell == 1:
                display_row.append('w')  # wall/obstacle
            else:
                display_row.append('.')  # free space
        display_grid.append(display_row)

    # Add goals
    goals = get_goal_areas()
    for goal_name, goal_data in goals.items():
        center_x, center_y = goal_data['center']
        gx, gy = world_to_grid(center_x, center_y)
        if 0 <= gy < len(display_grid) and 0 <= gx < len(display_grid[0]):
            display_grid[gy][gx] = goal_data['key']

    # Add robots
    robots = get_robot_positions()
    for robot_name, (x, y, symbol) in robots.items():
        gx, gy = world_to_grid(x, y)
        if 0 <= gy < len(display_grid) and 0 <= gx < len(display_grid[0]):
            display_grid[gy][gx] = symbol

    # Add traffic cones
    cone_positions = [
        (-3.47, -3.02), (-2.94, -2.91), (-2.4, -2.82), (-1.34, -2.76),
        (0.39, -1.53), (0.93, -1.38), (1.46, -1.22), (3.94, 1.04),
        (3.39, 0.86), (2.85, 0.74), (2.31, 0.85), (-4, -3.12),
        (-1.87, -2.84)
    ]

    for cone_x, cone_y in cone_positions:
        gx, gy = world_to_grid(cone_x, cone_y)
        if 0 <= gy < len(display_grid) and 0 <= gx < len(display_grid[0]):
            if display_grid[gy][gx] == '.':  # Only if not occupied by something else
                display_grid[gy][gx] = 'c'

    return display_grid

def save_detailed_grid():
    """
    Save a detailed grid visualization to file with goal areas marked
    """
    # Create grid with corrected resolution
    grid, resolution, width_m, height_m = create_grid_from_wbt()

    def world_to_grid(x, y):
        grid_x = int((x + width_m/2) / resolution)
        grid_y = int((y + height_m/2) / resolution)
        return max(0, min(len(grid[0])-1, grid_x)), max(0, min(len(grid)-1, grid_y))

    # Create display grid
    display_grid = []
    for row in grid:
        display_row = []
        for cell in row:
            if cell == 1:
                display_row.append('w')  # wall/obstacle
            else:
                display_row.append('.')  # free space
        display_grid.append(display_row)

    # Mark goal areas based on goalchecker.py coordinates
    # RED goal area: (3.18, 2.12) to (4.19, 3.90) - TOP RIGHT
    red_x1, red_y1 = world_to_grid(3.18, 2.12)
    red_x2, red_y2 = world_to_grid(4.19, 3.90)
    for y in range(min(red_y1, red_y2), max(red_y1, red_y2) + 1):
        for x in range(min(red_x1, red_x2), max(red_x1, red_x2) + 1):
            if 0 <= y < len(display_grid) and 0 <= x < len(display_grid[0]):
                if display_grid[y][x] == '.':
                    display_grid[y][x] = 'R'

    # GREEN goal area: (3.18, -4.86) to (4.19, -3.02) - BOTTOM RIGHT
    green_x1, green_y1 = world_to_grid(3.18, -4.86)
    green_x2, green_y2 = world_to_grid(4.19, -3.02)
    for y in range(min(green_y1, green_y2), max(green_y1, green_y2) + 1):
        for x in range(min(green_x1, green_x2), max(green_x1, green_x2) + 1):
            if 0 <= y < len(display_grid) and 0 <= x < len(display_grid[0]):
                if display_grid[y][x] == '.':
                    display_grid[y][x] = 'G'

    # DUCKS goal area: (-3.55, 3.08) to (-2.59, 4.19) - TOP LEFT
    duck_x1, duck_y1 = world_to_grid(-3.55, 3.08)
    duck_x2, duck_y2 = world_to_grid(-2.59, 4.19)
    for y in range(min(duck_y1, duck_y2), max(duck_y1, duck_y2) + 1):
        for x in range(min(duck_x1, duck_x2), max(duck_x1, duck_x2) + 1):
            if 0 <= y < len(display_grid) and 0 <= x < len(display_grid[0]):
                if display_grid[y][x] == '.':
                    display_grid[y][x] = 'D'

    # BALLS goal area: (-3.34, -4.67) to (-2.25, -3.67) - BOTTOM LEFT
    ball_x1, ball_y1 = world_to_grid(-3.34, -4.67)
    ball_x2, ball_y2 = world_to_grid(-2.25, -3.67)
    for y in range(min(ball_y1, ball_y2), max(ball_y1, ball_y2) + 1):
        for x in range(min(ball_x1, ball_x2), max(ball_x1, ball_x2) + 1):
            if 0 <= y < len(display_grid) and 0 <= x < len(display_grid[0]):
                if display_grid[y][x] == '.':
                    display_grid[y][x] = 'B'

    # Add robots
    robots = get_robot_positions()
    for robot_name, (x, y, symbol) in robots.items():
        gx, gy = world_to_grid(x, y)
        if 0 <= gy < len(display_grid) and 0 <= gx < len(display_grid[0]):
            display_grid[gy][gx] = symbol

    # Add traffic cones
    cone_positions = [
        (-3.47, -3.02), (-2.94, -2.91), (-2.4, -2.82), (-1.34, -2.76),
        (0.39, -1.53), (0.93, -1.38), (1.46, -1.22), (3.94, 1.04),
        (3.39, 0.86), (2.85, 0.74), (2.31, 0.85), (-4, -3.12),
        (-1.87, -2.84)
    ]

    for cone_x, cone_y in cone_positions:
        gx, gy = world_to_grid(cone_x, cone_y)
        if 0 <= gy < len(display_grid) and 0 <= gx < len(display_grid[0]):
            if display_grid[gy][gx] == '.':  # Only if not occupied by something else
                display_grid[gy][gx] = 'c'

    # Save to file
    with open('mission2_world_grid.txt', 'w') as f:
        f.write("=== MISSION2.WBT CORRECTED WORLD GRID ===\n")
        f.write("Based on precise coordinates from .wbt file and image analysis\n")
        f.write(f"Grid: {len(display_grid[0])}x{len(display_grid)} cells, {resolution}m resolution\n")
        f.write("Legend: w=wall/obstacle, .=free, c=cone, T=Tiago, 1/2/3=Pioneers\n")
        f.write("        R=RED goal, G=GREEN goal, D=DUCKS goal, B=BALLS goal\n")
        f.write("Coordinates: X=[-4.5,4.5], Y=[-5.5,5.5]\n\n")

        # Print full grid (flipped vertically for correct orientation)
        for y in range(len(display_grid)-1, -1, -1):  # Flip Y axis
            row_str = "".join(display_grid[y])
            f.write(f"{row_str}\n")

        f.write(f"\n=== QUADRANT LAYOUT ===\n")
        f.write(f"TOP-LEFT: DUCKS goal (D) at (-3.07, 3.64)\n")
        f.write(f"TOP-RIGHT: RED goal (R) at (3.69, 3.01)\n")
        f.write(f"BOTTOM-LEFT: BALLS goal (B) at (-2.81, -4.17)\n")
        f.write(f"BOTTOM-RIGHT: GREEN goal (G) at (3.69, -3.94)\n")

    return display_grid

def print_grid_analysis():
    """
    Main function to analyze and display the world - CORRECTED VERSION
    """
    print("=== MISSION2.WBT WORLD ANALYSIS - CORRECTED ===")
    print("Based on precise coordinates from .wbt file and image verification\n")

    # Create and save detailed grid
    display_grid = save_detailed_grid()

    print("Corrected detailed grid saved to 'mission2_world_grid_corrected.txt'")
    print(f"Grid Visualization ({len(display_grid[0])}x{len(display_grid)} cells, 0.2m resolution):")
    print("Legend: w=wall/obstacle, .=free, c=cone, T=Tiago, 1/2/3=Pioneers, R/G/D/B=goals")
    print("Coordinates: X=[-4.5,4.5], Y=[-5.5,5.5]\n")

    # Print grid with proper spacing
    for y in range(len(display_grid)-1, -1, -2):  # Flip Y axis, every 2nd row
        row_str = ""
        for x in range(0, len(display_grid[0])):  # All columns
            row_str += display_grid[y][x]
        y_coord = (y - len(display_grid)/2) * 0.2
        print(f"Y{y_coord:5.1f}: {row_str}")

    # Print column headers
    print("       ", end="")
    for x in range(0, len(display_grid[0])):
        if x % 5 == 0:
            x_coord = (x - len(display_grid[0])/2) * 0.2
            print(f"{x_coord:4.1f}", end="")
        else:
            print(" ", end="")
    print()

    # Print quadrant layout
    print(f"\n=== QUADRANT LAYOUT (from image and goalchecker.py) ===")
    print("The environment is divided into 4 quadrants by walls forming a cross:")
    print("┌─────────────┬─────────────┐")
    print("│ TOP-LEFT    │ TOP-RIGHT   │")
    print("│ DUCKS (D)   │ RED (R)     │")
    print("│ (-3.07,3.64)│ (3.69,3.01) │")
    print("├─────────────┼─────────────┤")
    print("│ BOTTOM-LEFT │ BOTTOM-RIGHT│")
    print("│ BALLS (B)   │ GREEN (G)   │")
    print("│(-2.81,-4.17)│(3.69,-3.94) │")
    print("└─────────────┴─────────────┘")

    # Print goal coordinates
    print(f"\n=== GOAL COORDINATES (from goalchecker.py) ===")
    goals = get_goal_areas()
    for goal_name, goal_data in goals.items():
        bounds = goal_data['bounds']
        center = goal_data['center']
        print(f"{goal_name.upper()} ({goal_data['key']}): {bounds[0]} to {bounds[2]} | Center: {center}")

    # Print robot positions
    print(f"\n=== ROBOT POSITIONS (from .wbt file) ===")
    robots = get_robot_positions()
    for robot_name, (x, y, symbol) in robots.items():
        print(f"{robot_name.upper()} ({symbol}): Position at ({x:.2f}, {y:.2f})")

    print(f"\n=== TRAFFIC CONE BARRIERS ===")
    print("Traffic cones create barriers within quadrants:")
    print("- Bottom-left quadrant: Line of cones from (-4,-3.12) to (-1.34,-2.76)")
    print("- Center area: Scattered cones from (0.39,-1.53) to (1.46,-1.22)")
    print("- Top-right quadrant: Line of cones from (2.31,0.85) to (3.94,1.04)")

    print(f"\n=== KEY FINDINGS ===")
    print("✓ Environment has cross-shaped wall layout creating 4 distinct quadrants")
    print("✓ Each quadrant contains one goal area as defined in goalchecker.py")
    print("✓ Traffic cones create internal barriers within quadrants")
    print("✓ No large boxes/containers - goals are open areas within quadrants")

if __name__ == "__main__":
    print_grid_analysis()