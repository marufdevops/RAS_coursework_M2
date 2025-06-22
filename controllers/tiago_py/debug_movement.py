"""
Debug Movement Controller

Simple test to understand robot movement directions and compass readings.
This will help diagnose the movement issue.
"""

from controller import Robot
import math

def test_robot_movement():
    """Test basic robot movement to understand coordinate system."""
    
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # Initialize devices
    l_motor = robot.getDevice("wheel_left_joint")
    r_motor = robot.getDevice("wheel_right_joint")
    compass = robot.getDevice("compass")
    gps = robot.getDevice('gps')
    
    # Enable devices
    compass.enable(timestep)
    gps.enable(timestep)
    
    # Set motor modes
    l_motor.setPosition(math.inf)
    r_motor.setPosition(math.inf)
    l_motor.setVelocity(0)
    r_motor.setVelocity(0)
    
    print("=== Robot Movement Debug Test ===")
    print("This test will help understand robot coordinate system")
    print("Press Ctrl+C to stop")
    
    step_count = 0
    test_phase = 0
    start_pos = None
    
    while robot.step(timestep) != -1:
        step_count += 1
        
        # Get current state
        pos = gps.getValues()[0:2]
        compass_values = compass.getValues()
        heading = math.atan2(compass_values[1], compass_values[0])
        
        if step_count % 50 == 0:  # Print every 50 steps (~0.5 seconds)
            print(f"Step {step_count}: Pos={pos[0]:.3f},{pos[1]:.3f}, Heading={heading:.3f}rad ({heading*180/math.pi:.1f}°)")
        
        # Test sequence
        if step_count == 100:  # Start test after 1 second
            start_pos = pos
            test_phase = 1
            print(f"\n--- Starting Movement Test ---")
            print(f"Initial position: {start_pos}")
            print(f"Initial heading: {heading:.3f} rad ({heading*180/math.pi:.1f}°)")
            
        elif test_phase == 1 and step_count < 300:  # Move forward for 2 seconds
            l_motor.setVelocity(1.0)
            r_motor.setVelocity(1.0)
            if step_count == 299:
                print(f"After forward movement: {pos}")
                dx = pos[0] - start_pos[0]
                dy = pos[1] - start_pos[1]
                print(f"Movement vector: dx={dx:.3f}, dy={dy:.3f}")
                print(f"Movement angle: {math.atan2(dy, dx):.3f} rad ({math.atan2(dy, dx)*180/math.pi:.1f}°)")
                test_phase = 2
                
        elif test_phase == 2 and step_count < 400:  # Stop
            l_motor.setVelocity(0)
            r_motor.setVelocity(0)
            if step_count == 399:
                test_phase = 3
                print(f"\n--- Testing Rotation ---")
                
        elif test_phase == 3 and step_count < 600:  # Rotate left
            l_motor.setVelocity(-1.0)
            r_motor.setVelocity(1.0)
            if step_count == 599:
                print(f"After left rotation: heading={heading:.3f} rad ({heading*180/math.pi:.1f}°)")
                test_phase = 4
                
        elif test_phase == 4 and step_count < 700:  # Stop
            l_motor.setVelocity(0)
            r_motor.setVelocity(0)
            if step_count == 699:
                test_phase = 5
                
        elif test_phase == 5 and step_count < 900:  # Move forward again
            l_motor.setVelocity(1.0)
            r_motor.setVelocity(1.0)
            if step_count == 899:
                print(f"After second forward movement: {pos}")
                dx2 = pos[0] - start_pos[0]
                dy2 = pos[1] - start_pos[1]
                print(f"Total movement vector: dx={dx2:.3f}, dy={dy2:.3f}")
                test_phase = 6
                
        elif test_phase == 6:  # Stop and finish
            l_motor.setVelocity(0)
            r_motor.setVelocity(0)
            if step_count == 950:
                print(f"\n--- Test Complete ---")
                print("Analysis:")
                print("- Positive motor speeds should move robot forward")
                print("- Left motor negative, right motor positive should rotate left")
                print("- Check if movement matches expected directions")
                break

if __name__ == "__main__":
    test_robot_movement()
