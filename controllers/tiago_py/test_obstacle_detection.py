#!/usr/bin/env python3
"""
Test script for simple obstacle detection logic
"""

import time

class MockNavigationController:
    """Mock navigation controller to test obstacle detection logic."""
    
    def __init__(self):
        # Simple obstacle detection variables
        self.obstacle_detected = False
        self.obstacle_pause_start = None
        self.obstacle_pause_duration = 10.0  # 10 seconds pause
        self.obstacle_threshold = 0.1  # 0.1m threshold
    
    def stop_robot(self):
        """Mock stop robot method."""
        print("🛑 Robot stopped")
    
    def check_obstacle_detection(self, mock_lidar_readings):
        """Simple obstacle detection: pause for 10 seconds if obstacle within 0.1m."""
        # Use mock LiDAR readings for testing
        lidar_values = mock_lidar_readings
        
        # Check front-facing sensors for obstacles
        # Use middle third of LiDAR readings (front-facing)
        front_readings = lidar_values[len(lidar_values)//3 : -len(lidar_values)//3]
        
        # Check if any reading is below threshold
        obstacle_detected_now = any(distance < self.obstacle_threshold for distance in front_readings)
        
        # If obstacle detected and we're not already pausing
        if obstacle_detected_now and not self.obstacle_detected:
            print(f"⚠️  OBSTACLE DETECTED within {self.obstacle_threshold}m! Pausing for {self.obstacle_pause_duration} seconds...")
            self.obstacle_detected = True
            self.obstacle_pause_start = time.time()
            self.stop_robot()
            return True  # Robot should pause
        
        # If we're currently pausing due to obstacle
        if self.obstacle_detected and self.obstacle_pause_start is not None:
            elapsed_time = time.time() - self.obstacle_pause_start
            if elapsed_time >= self.obstacle_pause_duration:
                print("✅ Obstacle pause complete. Resuming navigation...")
                self.obstacle_detected = False
                self.obstacle_pause_start = None
                return False  # Resume navigation
            else:
                # Still pausing
                remaining_time = self.obstacle_pause_duration - elapsed_time
                if int(remaining_time) != getattr(self, '_last_remaining', -1):
                    print(f"⏳ Obstacle pause: {remaining_time:.1f}s remaining...")
                    self._last_remaining = int(remaining_time)
                return True  # Continue pausing
        
        return False  # No obstacle, continue normal navigation


def test_obstacle_detection():
    """Test the obstacle detection logic."""
    print("Testing Simple Obstacle Detection Logic")
    print("=" * 50)
    
    controller = MockNavigationController()
    
    # Test 1: No obstacle (all readings > 0.1m)
    print("\n🧪 Test 1: No obstacle detected")
    no_obstacle_readings = [0.5, 0.3, 0.2, 0.15, 0.4, 0.6, 0.8]
    result = controller.check_obstacle_detection(no_obstacle_readings)
    print(f"Result: {'PAUSE' if result else 'CONTINUE'}")
    assert not result, "Should continue when no obstacle"
    
    # Test 2: Obstacle detected (reading < 0.1m)
    print("\n🧪 Test 2: Obstacle detected")
    obstacle_readings = [0.5, 0.3, 0.05, 0.15, 0.4, 0.6, 0.8]  # 0.05 < 0.1
    result = controller.check_obstacle_detection(obstacle_readings)
    print(f"Result: {'PAUSE' if result else 'CONTINUE'}")
    assert result, "Should pause when obstacle detected"
    
    # Test 3: During pause (should continue pausing)
    print("\n🧪 Test 3: During pause period")
    time.sleep(1)  # Wait 1 second
    result = controller.check_obstacle_detection(no_obstacle_readings)  # No obstacle now
    print(f"Result: {'PAUSE' if result else 'CONTINUE'}")
    assert result, "Should continue pausing during pause period"
    
    # Test 4: Simulate full pause duration
    print("\n🧪 Test 4: Simulating full pause duration...")
    controller.obstacle_pause_start = time.time() - 11  # Simulate 11 seconds ago
    result = controller.check_obstacle_detection(no_obstacle_readings)
    print(f"Result: {'PAUSE' if result else 'CONTINUE'}")
    assert not result, "Should resume after pause duration"
    
    print("\n✅ All tests passed! Simple obstacle detection logic works correctly.")
    print("\nSummary:")
    print("- Detects obstacles within 0.1m threshold ✓")
    print("- Pauses for 10 seconds when obstacle detected ✓") 
    print("- Resumes navigation after pause period ✓")
    print("- Provides clear status messages ✓")


if __name__ == "__main__":
    test_obstacle_detection()
