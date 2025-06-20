#!/usr/bin/env python3
"""
Test script for verifying balls waypoints in Webots
This script helps manually verify waypoint positions for the balls target
"""

def display_balls_waypoints():
    """Display the proposed waypoints for balls target"""
    print("=" * 60)
    print("BALLS TARGET WAYPOINT VERIFICATION")
    print("=" * 60)
    
    balls_waypoints = {
        'start': (-1.97, -1.96),        # Robot's initial spawn position
        'exit_start': (-1.5, -2.5),     # Move away from start area, heading southwest
        'west_approach': (-2.0, -3.0),   # Approach western area, avoiding central obstacles
        'pre_balls': (-2.3, -3.5),       # Position before final approach to balls
        'balls_approach': (-2.4, -3.8),  # Final approach position for balls detection
    }
    
    balls_target = (-2.5, -4.0)
    
    print(f"🎯 TARGET: Balls at {balls_target}")
    print("\n📍 PROPOSED WAYPOINTS:")
    
    for i, (name, coords) in enumerate(balls_waypoints.items(), 1):
        print(f"  {i}. {name:15} → {coords}")
    
    print("\n🛤️  NAVIGATION SEQUENCE:")
    sequence = ['start', 'exit_start', 'west_approach', 'pre_balls', 'balls_approach']
    for i in range(len(sequence) - 1):
        from_wp = sequence[i]
        to_wp = sequence[i + 1]
        from_coords = balls_waypoints[from_wp]
        to_coords = balls_waypoints[to_wp]
        distance = ((to_coords[0] - from_coords[0])**2 + (to_coords[1] - from_coords[1])**2)**0.5
        print(f"  {from_wp} → {to_wp} (distance: {distance:.2f}m)")
    
    # Calculate final distance to target
    final_coords = balls_waypoints['balls_approach']
    target_distance = ((balls_target[0] - final_coords[0])**2 + (balls_target[1] - final_coords[1])**2)**0.5
    print(f"\n🎯 Final approach distance to target: {target_distance:.2f}m")
    
    return balls_waypoints

def generate_verification_instructions():
    """Generate step-by-step verification instructions"""
    print("\n" + "=" * 60)
    print("MANUAL VERIFICATION INSTRUCTIONS")
    print("=" * 60)
    
    instructions = [
        "1. 🚀 START WEBOTS",
        "   - Open mission2.wbt world file",
        "   - Ensure TiaGo robot is at start position (-1.97, -1.96)",
        "",
        "2. 🧭 VERIFY EACH WAYPOINT",
        "   For each waypoint, manually drive the robot and check:",
        "   ✅ Position is in open space (no walls/obstacles)",
        "   ✅ Robot can reach position without collisions", 
        "   ✅ Good visibility/clearance around the position",
        "   ✅ Reasonable distance from previous waypoint",
        "",
        "3. 📝 WAYPOINT VERIFICATION CHECKLIST",
        "   □ exit_start (-1.5, -2.5)    - Clear path from start?",
        "   □ west_approach (-2.0, -3.0) - Avoids central obstacles?", 
        "   □ pre_balls (-2.3, -3.5)     - Safe approach position?",
        "   □ balls_approach (-2.4, -3.8) - Can detect balls target?",
        "",
        "4. 🎯 TARGET DETECTION TEST",
        "   - Navigate to balls_approach waypoint",
        "   - Verify robot can detect balls target at (-2.5, -4.0)",
        "   - Check distance is within detection range (~0.8m)",
        "",
        "5. 🔄 REPORT RESULTS",
        "   For each waypoint, report:",
        "   ✅ GOOD - Position works as intended",
        "   ⚠️  ADJUST - Position needs minor adjustment (suggest new coords)",
        "   ❌ BAD - Position blocked/unsafe (suggest alternative)",
    ]
    
    for instruction in instructions:
        print(instruction)

def generate_test_navigation_code():
    """Generate code snippet for testing navigation in Webots"""
    print("\n" + "=" * 60)
    print("TEST NAVIGATION CODE SNIPPET")
    print("=" * 60)
    
    code = '''
# Add this to your TiaGo controller for manual waypoint testing:

def test_balls_waypoints():
    """Test navigation to each balls waypoint manually"""
    balls_waypoints = {
        'start': (-1.97, -1.96),
        'exit_start': (-1.5, -2.5),
        'west_approach': (-2.0, -3.0),
        'pre_balls': (-2.3, -3.5),
        'balls_approach': (-2.4, -3.8),
    }
    
    current_test = 'exit_start'  # Change this to test different waypoints
    target_coord = balls_waypoints[current_test]
    
    print(f"Testing waypoint: {current_test} at {target_coord}")
    
    # Use your existing move_to_waypoint_direct function
    if move_to_waypoint_direct(target_coord):
        print(f"✅ Successfully reached {current_test}")
        pos = get_current_position()
        distance = math.sqrt((pos[0] - target_coord[0])**2 + (pos[1] - target_coord[1])**2)
        print(f"Final distance from target: {distance:.3f}m")
    else:
        print(f"❌ Failed to reach {current_test}")

# Usage in main loop:
# 1. Set current_test to the waypoint you want to test
# 2. Run the controller
# 3. Observe robot behavior and note any issues
# 4. Report results for each waypoint
'''
    
    print(code)

def generate_pddl_test():
    """Generate PDDL test for balls waypoints"""
    print("\n" + "=" * 60)
    print("PDDL PLANNING TEST")
    print("=" * 60)
    
    print("Once waypoints are verified, test PDDL planning:")
    print()
    print("1. 📝 Update GOAL_WAYPOINTS['balls']['verified'] = True")
    print("2. 🔧 Run PDDL planner:")
    print("   python -c \"from tiago_py import write_pddl_problem_goal_specific; write_pddl_problem_goal_specific('balls')\"")
    print("3. 🧠 Generate solution:")
    print("   pyperplan -H hff -s astar warehouse_domain.pddl warehouse_problem.pddl")
    print("4. 📋 Check solution:")
    print("   cat warehouse_problem.pddl.soln")
    print()
    print("Expected solution sequence:")
    print("(move robot start exit_start)")
    print("(move robot exit_start west_approach)")
    print("(move robot west_approach pre_balls)")
    print("(move robot pre_balls balls_approach)")

def main():
    """Main verification workflow"""
    print("🏀 BALLS WAYPOINT VERIFICATION SYSTEM")
    
    # Display proposed waypoints
    waypoints = display_balls_waypoints()
    
    # Show verification instructions
    generate_verification_instructions()
    
    # Provide test code
    generate_test_navigation_code()
    
    # Show PDDL testing steps
    generate_pddl_test()
    
    print("\n" + "=" * 60)
    print("NEXT STEPS")
    print("=" * 60)
    print("1. 🧪 Test each waypoint manually in Webots")
    print("2. 📝 Report verification results")
    print("3. 🔧 Adjust waypoints based on feedback")
    print("4. ✅ Mark as verified when all waypoints work")
    print("5. 🎯 Proceed to next target (red, green, or ducks)")
    print()
    print("💡 TIP: Start with 'exit_start' waypoint and work your way through the sequence")
    print("🔍 Focus on obstacle avoidance and smooth navigation between waypoints")

if __name__ == "__main__":
    main()
