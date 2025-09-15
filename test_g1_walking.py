#!/usr/bin/env python3
"""
Simple test script for G1 humanoid walking patterns.
Tests basic walking movements without full simulation.
"""

import torch
import math
import time

class G1WalkingTestController:
    """Test controller to validate G1 walking patterns."""
    
    def __init__(self):
        self.device = torch.device('cpu')
        
        # Joint names from G1_CFG
        self.joint_names = [
            "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint", 
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "torso_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_pitch_joint", "left_elbow_roll_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_pitch_joint", "right_elbow_roll_joint"
        ]
        
        # Default positions from G1_CFG
        default_pos_dict = {
            "left_hip_pitch_joint": -0.20,
            "right_hip_pitch_joint": -0.20,
            "left_knee_joint": 0.42,
            "right_knee_joint": 0.42,
            "left_ankle_pitch_joint": -0.23,
            "right_ankle_pitch_joint": -0.23,
            "left_hip_roll_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "left_hip_yaw_joint": 0.0,
            "right_hip_yaw_joint": 0.0,
            "left_ankle_roll_joint": 0.0,
            "right_ankle_roll_joint": 0.0,
            "torso_joint": 0.0,
            "left_shoulder_pitch_joint": 0.35,
            "right_shoulder_pitch_joint": 0.35,
            "left_shoulder_roll_joint": 0.16,
            "right_shoulder_roll_joint": -0.16,
            "left_shoulder_yaw_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "left_elbow_pitch_joint": 0.87,
            "right_elbow_pitch_joint": 0.87,
            "left_elbow_roll_joint": 0.0,
            "right_elbow_roll_joint": 0.0,
        }
        
        # Create default position vector
        positions = []
        for joint_name in self.joint_names:
            if joint_name in default_pos_dict:
                positions.append(default_pos_dict[joint_name])
            else:
                positions.append(0.0)
                
        self.default_joint_pos = torch.tensor(positions, dtype=torch.float32, device=self.device)
        print(f"Initialized G1 test controller with {len(self.joint_names)} joints")
        
        # Walking parameters
        self.walking_frequency = 2.0
        self.walking_phase = 0.0
        
    def generate_walking_pattern(self, vel_cmd, dt=0.02):
        """Generate walking pattern based on velocity commands."""
        vel_x, vel_y, vel_yaw = vel_cmd[0], vel_cmd[1], vel_cmd[2]
        
        # Check if we should be walking
        vel_magnitude = math.sqrt(vel_x**2 + vel_y**2)
        is_walking = vel_magnitude > 0.1 or abs(vel_yaw) > 0.1
        
        if not is_walking:
            return self.default_joint_pos.clone()
        
        # Update walking phase
        self.walking_phase += self.walking_frequency * dt
        if self.walking_phase > 2 * math.pi:
            self.walking_phase -= 2 * math.pi
            
        # Start with default positions
        joint_pos = self.default_joint_pos.clone()
        
        # Simple walking pattern
        left_leg_phase = math.sin(self.walking_phase)
        right_leg_phase = math.sin(self.walking_phase + math.pi)
        
        # Map joint names to indices
        joint_idx = {name: i for i, name in enumerate(self.joint_names)}
        
        # Modify leg joints for walking
        if "left_hip_pitch_joint" in joint_idx:
            joint_pos[joint_idx["left_hip_pitch_joint"]] += vel_x * 0.2 * left_leg_phase
        if "right_hip_pitch_joint" in joint_idx:
            joint_pos[joint_idx["right_hip_pitch_joint"]] += vel_x * 0.2 * right_leg_phase
            
        if "left_knee_joint" in joint_idx:
            joint_pos[joint_idx["left_knee_joint"]] += max(0, left_leg_phase) * 0.3
        if "right_knee_joint" in joint_idx:
            joint_pos[joint_idx["right_knee_joint"]] += max(0, right_leg_phase) * 0.3
            
        # Add yaw rotation
        if "left_hip_yaw_joint" in joint_idx:
            joint_pos[joint_idx["left_hip_yaw_joint"]] += vel_yaw * 0.1
        if "right_hip_yaw_joint" in joint_idx:
            joint_pos[joint_idx["right_hip_yaw_joint"]] -= vel_yaw * 0.1
            
        # Add lateral movement
        if "left_hip_roll_joint" in joint_idx:
            joint_pos[joint_idx["left_hip_roll_joint"]] += vel_y * 0.15
        if "right_hip_roll_joint" in joint_idx:
            joint_pos[joint_idx["right_hip_roll_joint"]] -= vel_y * 0.15
            
        return joint_pos
    
    def test_movements(self):
        """Test various movement patterns."""
        dt = 0.02  # 20ms timestep
        
        test_cases = [
            ("Standing", [0.0, 0.0, 0.0]),
            ("Forward", [1.0, 0.0, 0.0]),
            ("Backward", [-1.0, 0.0, 0.0]),
            ("Left strafe", [0.0, 1.0, 0.0]),
            ("Right strafe", [0.0, -1.0, 0.0]),
            ("Turn left", [0.0, 0.0, 1.0]),
            ("Turn right", [0.0, 0.0, -1.0]),
        ]
        
        for test_name, vel_cmd in test_cases:
            print(f"\n--- Testing {test_name} movement ---")
            print(f"Velocity command: x={vel_cmd[0]}, y={vel_cmd[1]}, yaw={vel_cmd[2]}")
            
            # Reset phase for each test
            self.walking_phase = 0.0
            
            # Test for 2 seconds (100 timesteps)
            for step in range(100):
                joint_positions = self.generate_walking_pattern(vel_cmd, dt)
                
                # Print key joint positions every 20 steps
                if step % 20 == 0:
                    joint_idx = {name: i for i, name in enumerate(self.joint_names)}
                    left_hip_pitch = joint_positions[joint_idx["left_hip_pitch_joint"]].item()
                    right_hip_pitch = joint_positions[joint_idx["right_hip_pitch_joint"]].item()
                    left_knee = joint_positions[joint_idx["left_knee_joint"]].item()
                    right_knee = joint_positions[joint_idx["right_knee_joint"]].item()
                    
                    print(f"  Step {step:2d}: L_hip={left_hip_pitch:6.3f}, R_hip={right_hip_pitch:6.3f}, "
                          f"L_knee={left_knee:6.3f}, R_knee={right_knee:6.3f}")
            
            time.sleep(0.5)  # Pause between tests

def main():
    print("G1 Humanoid Walking Pattern Test")
    print("================================")
    
    controller = G1WalkingTestController()
    controller.test_movements()
    
    print("\n--- Test completed ---")
    print("If you see alternating joint values for forward movement,")
    print("the walking pattern is working correctly!")

if __name__ == "__main__":
    main()