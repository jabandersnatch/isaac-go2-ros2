import os

import carb
import gymnasium as gym
import torch
from isaaclab.envs import DirectRLEnv, ManagerBasedEnv, ManagerBasedRLEnv

base_vel_cmd_input = None

# Arm control commands
arm_cmd_left_shoulder = 0.0
arm_cmd_right_shoulder = 0.0
arm_cmd_elbow = 0.0


# Initialize base_vel_cmd_input as a tensor when created
def init_base_vel_cmd(num_envs):
    global base_vel_cmd_input
    base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)


# Modify base_vel_cmd to use the tensor directly
def base_vel_cmd(env: ManagerBasedEnv) -> torch.Tensor:
    global base_vel_cmd_input
    if base_vel_cmd_input is None:
        # Initialize if not already done (fallback for environment initialization)
        num_envs = getattr(env.scene, "num_envs", 1)
        base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)
    return base_vel_cmd_input.clone().to(env.device)


# G1 humanoid-specific keyboard controls
def sub_keyboard_event(event) -> bool:
    global base_vel_cmd_input, arm_cmd_left_shoulder, arm_cmd_right_shoulder, arm_cmd_elbow
    # Reduced velocities to prevent falling
    lin_vel = 0.2
    ang_vel = 0.3

    if base_vel_cmd_input is not None:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # Update tensor values for environment 0
            if event.input.name == "W":
                base_vel_cmd_input[0] = torch.tensor(
                    [lin_vel, 0, 0], dtype=torch.float32
                )
            elif event.input.name == "S":
                base_vel_cmd_input[0] = torch.tensor(
                    [-lin_vel, 0, 0], dtype=torch.float32
                )
            elif event.input.name == "A":
                base_vel_cmd_input[0] = torch.tensor(
                    [0, lin_vel, 0], dtype=torch.float32
                )
            elif event.input.name == "D":
                base_vel_cmd_input[0] = torch.tensor(
                    [0, -lin_vel, 0], dtype=torch.float32
                )
            elif event.input.name == "Z":
                base_vel_cmd_input[0] = torch.tensor(
                    [0, 0, ang_vel], dtype=torch.float32
                )
            elif event.input.name == "C":
                base_vel_cmd_input[0] = torch.tensor(
                    [0, 0, -ang_vel], dtype=torch.float32
                )

            # Arm controls (Q/E for shoulder, R/T for elbow)  
            elif event.input.name == "Q":
                # Left shoulder up
                base_vel_cmd_input[0] = torch.tensor(
                    [0, 0, 0], dtype=torch.float32
                )  # Clear movement
                arm_cmd_left_shoulder = 0.2
            elif event.input.name == "E":
                # Right shoulder up
                base_vel_cmd_input[0] = torch.tensor([0, 0, 0], dtype=torch.float32)
                arm_cmd_right_shoulder = 0.2
            elif event.input.name == "R":
                # Both elbows bend
                base_vel_cmd_input[0] = torch.tensor([0, 0, 0], dtype=torch.float32)
                arm_cmd_elbow = 0.3
            elif event.input.name == "T":
                # Both elbows extend
                base_vel_cmd_input[0] = torch.tensor([0, 0, 0], dtype=torch.float32)
                arm_cmd_elbow = -0.3

        # Reset commands to zero on key release
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            base_vel_cmd_input.zero_()
            arm_cmd_left_shoulder = 0.0
            arm_cmd_right_shoulder = 0.0
            arm_cmd_elbow = 0.0
    return True


class G1BasicController:
    """Enhanced joint position controller for G1 humanoid with velocity-based movement."""

    def __init__(self, env):
        self.env = env
        self.device = env.device

        # Get robot information
        self.robot = env.scene["unitree_g1"]
        self.joint_names = self.robot.data.joint_names
        self.num_joints = len(self.joint_names)

        # Initialize default positions
        self.default_joint_pos = self._get_default_joint_positions()

    def _get_default_joint_positions(self):
        """Get default standing joint positions for G1."""
        # Standard Isaac Lab G1 default positions
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
        }

        # Create position vector matching joint order
        positions = []
        for joint_name in self.joint_names:
            if joint_name in default_pos_dict:
                positions.append(default_pos_dict[joint_name])
            else:
                positions.append(0.0)

        return torch.tensor(positions, dtype=torch.float32, device=self.device)

    def _generate_simple_movement(self, vel_cmd):
        """Generate simple joint movements based on velocity commands - for testing."""
        # Extract velocity components
        vel_x = vel_cmd[0]
        vel_y = vel_cmd[1]
        vel_yaw = vel_cmd[2]

        # Start with default positions
        joint_pos = self.default_joint_pos.clone()

        # Map joint names to indices for easier manipulation
        joint_idx = {}
        for i, name in enumerate(self.joint_names):
            joint_idx[name] = i

        # Small movements with side-fall bias
        movement_scale = 0.08  # Slightly larger for more visible movement

        # Forward (W) - subtle weight shift forward with balance
        if vel_x > 0.1:
            if "left_hip_pitch_joint" in joint_idx:
                joint_pos[joint_idx["left_hip_pitch_joint"]] += movement_scale * 2.0
            if "right_hip_pitch_joint" in joint_idx:
                joint_pos[joint_idx["right_hip_pitch_joint"]] += movement_scale * 2.0
            # Counter-balance with arms backward
            if "left_shoulder_pitch_joint" in joint_idx:
                joint_pos[joint_idx["left_shoulder_pitch_joint"]] -= movement_scale * 1.0
            if "right_shoulder_pitch_joint" in joint_idx:
                joint_pos[joint_idx["right_shoulder_pitch_joint"]] -= movement_scale * 1.0

        # Backward (S) - subtle weight shift back with balance
        elif vel_x < -0.1:
            if "left_hip_pitch_joint" in joint_idx:
                joint_pos[joint_idx["left_hip_pitch_joint"]] -= movement_scale * 1.5
            if "right_hip_pitch_joint" in joint_idx:
                joint_pos[joint_idx["right_hip_pitch_joint"]] -= movement_scale * 1.5
            # Counter-balance with arms forward
            if "left_shoulder_pitch_joint" in joint_idx:
                joint_pos[joint_idx["left_shoulder_pitch_joint"]] += movement_scale * 1.0
            if "right_shoulder_pitch_joint" in joint_idx:
                joint_pos[joint_idx["right_shoulder_pitch_joint"]] += movement_scale * 1.0

        # Left strafe (A) - shift weight to right foot, lean left
        if vel_y > 0.1:
            if "left_hip_roll_joint" in joint_idx:
                joint_pos[joint_idx["left_hip_roll_joint"]] += movement_scale * 1.0
            if "right_hip_roll_joint" in joint_idx:
                joint_pos[joint_idx["right_hip_roll_joint"]] += movement_scale * 1.0
            # Counter-balance with right arm
            if "right_shoulder_roll_joint" in joint_idx:
                joint_pos[joint_idx["right_shoulder_roll_joint"]] += movement_scale * 2.0

        # Right strafe (D) - shift weight to left foot, lean right  
        elif vel_y < -0.1:
            if "left_hip_roll_joint" in joint_idx:
                joint_pos[joint_idx["left_hip_roll_joint"]] -= movement_scale * 1.0
            if "right_hip_roll_joint" in joint_idx:
                joint_pos[joint_idx["right_hip_roll_joint"]] -= movement_scale * 1.0
            # Counter-balance with left arm
            if "left_shoulder_roll_joint" in joint_idx:
                joint_pos[joint_idx["left_shoulder_roll_joint"]] -= movement_scale * 2.0

        # Turn left (Z) - small yaw with arm counter-balance
        if vel_yaw > 0.1:
            if "left_hip_yaw_joint" in joint_idx:
                joint_pos[joint_idx["left_hip_yaw_joint"]] += movement_scale * 0.5
            if "right_hip_yaw_joint" in joint_idx:
                joint_pos[joint_idx["right_hip_yaw_joint"]] -= movement_scale * 0.5
            # Counter-rotate with arms for balance
            if "left_shoulder_yaw_joint" in joint_idx:
                joint_pos[joint_idx["left_shoulder_yaw_joint"]] -= movement_scale * 1.0

        # Turn right (C) - small yaw with arm counter-balance
        elif vel_yaw < -0.1:
            if "left_hip_yaw_joint" in joint_idx:
                joint_pos[joint_idx["left_hip_yaw_joint"]] -= movement_scale * 0.5
            if "right_hip_yaw_joint" in joint_idx:
                joint_pos[joint_idx["right_hip_yaw_joint"]] += movement_scale * 0.5
            # Counter-rotate with arms for balance  
            if "right_shoulder_yaw_joint" in joint_idx:
                joint_pos[joint_idx["right_shoulder_yaw_joint"]] += movement_scale * 1.0


        # Apply arm controls
        global arm_cmd_left_shoulder, arm_cmd_right_shoulder, arm_cmd_elbow

        # Left shoulder control (Q key)
        if "left_shoulder_pitch_joint" in joint_idx:
            joint_pos[joint_idx["left_shoulder_pitch_joint"]] += arm_cmd_left_shoulder

        # Right shoulder control (E key)
        if "right_shoulder_pitch_joint" in joint_idx:
            joint_pos[joint_idx["right_shoulder_pitch_joint"]] += arm_cmd_right_shoulder

        # Elbow controls (R/T keys)
        if "left_elbow_pitch_joint" in joint_idx:
            joint_pos[joint_idx["left_elbow_pitch_joint"]] += arm_cmd_elbow
        if "right_elbow_pitch_joint" in joint_idx:
            joint_pos[joint_idx["right_elbow_pitch_joint"]] += arm_cmd_elbow

        return joint_pos

    def get_actions(self, obs=None):
        """Get joint position actions with velocity-based movement.

        Args:
            obs: Environment observations (currently unused, but available for future RL policy integration)
        """
        num_envs = self.env.scene.num_envs

        # Get velocity commands for the current environment
        global base_vel_cmd_input
        if base_vel_cmd_input is not None and base_vel_cmd_input.numel() > 0:
            vel_cmd = base_vel_cmd_input[0].to(self.device)
        else:
            vel_cmd = torch.zeros(3, device=self.device)

        # Generate movement pattern
        joint_positions = self._generate_simple_movement(vel_cmd)

        # Expand for all environments
        actions = joint_positions.unsqueeze(0).repeat(num_envs, 1)

        return actions


def get_g1_basic_controller(cfg):
    """Create G1 environment with basic joint controller."""
    # Create real Isaac Lab environment like Go2 does
    env = ManagerBasedRLEnv(cfg=cfg)

    # Reset environment to initialize simulation
    obs, _ = env.reset()

    # Create controller
    controller = G1BasicController(env)

    return env, controller

