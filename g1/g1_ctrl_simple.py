import os
import torch
import carb
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

base_vel_cmd_input = None

# Initialize base_vel_cmd_input as a tensor when created
def init_base_vel_cmd(num_envs):
    global base_vel_cmd_input
    base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)

# Modify base_vel_cmd to use the tensor directly
def base_vel_cmd(env) -> torch.Tensor:
    global base_vel_cmd_input
    if base_vel_cmd_input is None:
        # Initialize if not already done (fallback for environment initialization)
        num_envs = getattr(env.scene, 'num_envs', 1)
        base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)
    return base_vel_cmd_input.clone().to(env.device)

# G1 humanoid-specific keyboard controls
def sub_keyboard_event(event) -> bool:
    global base_vel_cmd_input
    # Slower velocities for humanoid walking
    lin_vel = 0.5
    ang_vel = 0.8
    
    if base_vel_cmd_input is not None:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # Update tensor values for environment 0
            if event.input.name == 'W':
                base_vel_cmd_input[0] = torch.tensor([lin_vel, 0, 0], dtype=torch.float32)
            elif event.input.name == 'S':
                base_vel_cmd_input[0] = torch.tensor([-lin_vel, 0, 0], dtype=torch.float32)
            elif event.input.name == 'A':
                base_vel_cmd_input[0] = torch.tensor([0, lin_vel, 0], dtype=torch.float32)
            elif event.input.name == 'D':
                base_vel_cmd_input[0] = torch.tensor([0, -lin_vel, 0], dtype=torch.float32)
            elif event.input.name == 'Z':
                base_vel_cmd_input[0] = torch.tensor([0, 0, ang_vel], dtype=torch.float32)
            elif event.input.name == 'C':
                base_vel_cmd_input[0] = torch.tensor([0, 0, -ang_vel], dtype=torch.float32)
        
        # Reset commands to zero on key release
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            base_vel_cmd_input.zero_()
    return True

class SimpleG1Controller:
    """Minimal G1 controller that just holds default positions - like Go2 policy."""
    
    def __init__(self, env):
        logger.info("[G1_SIMPLE] Initializing SimpleG1Controller")
        self.env = env
        self.device = env.device
        
        # Get robot information
        self.robot = env.scene["unitree_g1"]
        self.num_joints = 37  # G1 has 37 joints
        
        # Simple default standing pose (no complex calculations)
        self.default_actions = torch.zeros((1, self.num_joints), device=self.device)
        
        logger.info(f"[G1_SIMPLE] Controller initialized with {self.num_joints} joints")
    
    def __call__(self, obs):
        """Act like Go2 policy - simple function call with observations."""
        # Just return default actions (standing pose) - like Go2 policy does
        num_envs = obs['policy'].shape[0] if isinstance(obs, dict) else obs.shape[0]
        return self.default_actions.repeat(num_envs, 1)

def get_g1_simple_controller(cfg):
    """Create G1 environment with simple controller - exactly like Go2 pattern."""
    logger.info("[G1_SIMPLE] Creating G1 environment...")
    
    # Import here to avoid circular imports
    from isaaclab.envs import ManagerBasedRLEnv
    
    try:
        # Create environment exactly like Go2
        env = ManagerBasedRLEnv(cfg=cfg)
        logger.info("[G1_SIMPLE] Environment created successfully")
        
        # Reset environment
        obs, _ = env.reset()
        logger.info("[G1_SIMPLE] Environment reset successful")
        
        # Create simple controller that acts like Go2 policy
        controller = SimpleG1Controller(env)
        logger.info("[G1_SIMPLE] Simple controller created successfully")
        
        return env, controller
        
    except Exception as e:
        logger.error(f"[G1_SIMPLE] Failed to create G1 environment: {str(e)}")
        import traceback
        logger.error(f"[G1_SIMPLE] Traceback: {traceback.format_exc()}")
        raise