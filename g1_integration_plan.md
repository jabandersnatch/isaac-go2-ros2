# Implementation Plan: Unitree G1 Humanoid Robot in Isaac Sim/ROS2 Framework

## Executive Summary

This plan details how to add support for the Unitree G1 humanoid robot to your existing Isaac Sim/ROS2 framework while maintaining the current architecture and patterns used for the Go2 quadruped. The G1 integration will follow the same modular structure but with adaptations for bipedal locomotion and the robot's 35 degrees of freedom.

## Phase 1: Initial Setup and Model Import (Week 1)

### 1.1 Directory Structure

Create parallel directory structure for G1:

```bash
# Create G1-specific directories mirroring Go2 structure
mkdir -p g1
mkdir -p ckpts/unitree_g1
mkdir -p cfg/g1_configs
```

### 1.2 Robot Model Import

**Actions:**

1. Obtain Unitree G1 URDF/MJCF model
2. Import to Isaac Sim using URDF Importer extension
3. Convert to USD format and save to assets
4. Validate all 35 joints and physics properties

**Key Configuration for URDF Import:**

- Uncheck "Fixed Base Link"
- Set "Joint Drive Type" to "Velocity"
- Verify joint limits and ranges for all 35 DoF

### 1.3 Configuration Files

Create `cfg/g1_sim.yaml`:

```yaml
num_envs: 1
freq: 50 # Hz
env_name: "warehouse"
camera_follow: true

robot:
  type: "unitree_g1"
  dof: 35

sensor:
  enable_lidar: true
  enable_camera: true
  color_image: true
  depth_image: true
  semantic_segmentation: true
```

## Phase 2: Robot Environment Configuration (Week 2)

### 2.1 Create G1 Environment Class

Create `g1/g1_env.py` following the Go2 pattern:

```python
from isaaclab_assets.robots.unitree import UNITREE_G1_CFG  # To be created

@configclass
class G1SimCfg(InteractiveSceneCfg):
    # Ground plane (same as Go2)
    ground = AssetBaseCfg(...)

    # G1 Robot with 35 DoF
    unitree_g1: ArticulationCfg = UNITREE_G1_CFG.replace(
        prim_path="{ENV_REGEX_NS}/G1"
    )

    # Foot contact sensors (bipedal specific)
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/G1/.*_foot",
        history_length=3,
        track_air_time=True
    )

    # Height scanner for terrain
    height_scanner = RayCasterCfg(...)
```

### 2.2 Observation Configuration

Adapt observations for bipedal control:

```python
@configclass
class ObservationsCfg:
    class PolicyCfg(ObsGroup):
        # Base state
        base_lin_vel = ObsTerm(...)
        base_ang_vel = ObsTerm(...)
        projected_gravity = ObsTerm(...)

        # Joint state (35 joints)
        joint_pos = ObsTerm(...)
        joint_vel = ObsTerm(...)

        # Bipedal-specific
        center_of_mass = ObsTerm(...)  # Critical for balance
        foot_contacts = ObsTerm(...)   # Left/right foot
        base_height = ObsTerm(...)     # Height from ground
```

## Phase 3: Control System Adaptation (Week 3)

### 3.1 Create G1 Controller

Create `g1/g1_ctrl.py`:

```python
import torch
from rsl_rl.runners import OnPolicyRunner

base_vel_cmd_input = None

def init_base_vel_cmd(num_envs):
    global base_vel_cmd_input
    base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)

def get_g1_standing_policy(cfg):
    """Load policy for standing/balancing"""
    env = gym.make("Isaac-Standing-Unitree-G1-v0", cfg=cfg)
    # Load checkpoint for standing
    ckpt_path = "ckpts/unitree_g1/standing_model.pt"
    return load_policy(env, ckpt_path)

def get_g1_walking_policy(cfg):
    """Load policy for bipedal walking"""
    env = gym.make("Isaac-Walking-Unitree-G1-v0", cfg=cfg)
    ckpt_path = "ckpts/unitree_g1/walking_model.pt"
    return load_policy(env, ckpt_path)
```

### 3.2 Control Configuration

Create `g1/g1_ctrl_cfg.py`:

```python
unitree_g1_walking_cfg = {
    'device': 'cuda:0',
    'num_steps_per_env': 24,
    'policy': {
        'actor_hidden_dims': [512, 512, 256],  # Larger network for 35 DoF
        'critic_hidden_dims': [512, 512, 256],
    },
    'algorithm': {
        'class_name': 'PPO',
        'learning_rate': 0.0003,  # Lower LR for stability
        'clip_param': 0.2,
    },
    'load_run': 'unitree_g1',
    'load_checkpoint': 'walking_model.pt'
}
```

## Phase 4: Sensor Integration (Week 4)

### 4.1 Adapt Sensor Manager

Create `g1/g1_sensors.py`:

```python
class G1SensorManager:
    def __init__(self, num_envs):
        self.num_envs = num_envs

    def add_rtx_lidar(self):
        # Mount LiDAR at torso height (~0.8m)
        for env_idx in range(self.num_envs):
            sensor = create_lidar(
                parent=f"/World/envs/env_{env_idx}/G1/torso",
                translation=(0.0, 0.0, 0.0),  # Center of torso
            )

    def add_cameras(self):
        # Head-mounted camera for G1
        camera = Camera(
            prim_path=f"...G1/head/camera",
            translation=[0.1, 0.0, 0.0],  # Front of head
        )
```

## Phase 5: ROS2 Bridge Adaptation (Week 5)

### 5.1 Create G1 ROS2 Bridge

Adapt `ros2/g1_ros2_bridge.py`:

```python
class G1DataManager(Node):
    def __init__(self, env, lidar_annotators, cameras, cfg):
        super().__init__("g1_data_manager")

        # Publishers for G1-specific data
        self.joint_state_pub = self.create_publisher(
            JointState, '/unitree_g1/joint_states', 10
        )

        # Center of mass publisher (important for bipeds)
        self.com_pub = self.create_publisher(
            PointStamped, '/unitree_g1/center_of_mass', 10
        )

        # Foot contact publishers
        self.foot_contact_pub = self.create_publisher(
            ContactsState, '/unitree_g1/foot_contacts', 10
        )

    def publish_joint_states(self):
        """Publish all 35 joint states"""
        msg = JointState()
        msg.name = [f"joint_{i}" for i in range(35)]
        msg.position = self.env.joint_positions.tolist()
        msg.velocity = self.env.joint_velocities.tolist()
        self.joint_state_pub.publish(msg)
```

## Phase 6: Main Integration Script (Week 6)

### 6.1 Create Main Entry Point

Create `isaac_g1_ros2.py`:

```python
import hydra
import rclpy
from g1.g1_env import G1HumanoidEnvCfg
import g1.g1_ctrl as g1_ctrl
import g1.g1_sensors as g1_sensors
import ros2.g1_ros2_bridge as g1_ros2_bridge

@hydra.main(config_path="cfg", config_name="g1_sim")
def run_g1_simulator(cfg):
    # G1 Environment setup
    g1_env_cfg = G1HumanoidEnvCfg()
    g1_env_cfg.scene.num_envs = cfg.num_envs

    # Initialize control
    g1_ctrl.init_base_vel_cmd(cfg.num_envs)

    # Load appropriate policy
    if cfg.robot.mode == "standing":
        env, policy = g1_ctrl.get_g1_standing_policy(g1_env_cfg)
    else:
        env, policy = g1_ctrl.get_g1_walking_policy(g1_env_cfg)

    # Create environment (reuse existing environments)
    if cfg.env_name == "warehouse":
        sim_env.create_warehouse_env()

    # Sensor setup
    sm = g1_sensors.G1SensorManager(cfg.num_envs)
    lidar_annotators = sm.add_rtx_lidar()
    cameras = sm.add_cameras()

    # ROS2 Bridge
    rclpy.init()
    dm = g1_ros2_bridge.G1DataManager(env, lidar_annotators, cameras, cfg)

    # Main loop
    obs, _ = env.reset()
    while simulation_app.is_running():
        actions = policy(obs)
        obs, _, _, _ = env.step(actions)
        dm.pub_ros2_data()
        rclpy.spin_once(dm)
```

## Phase 7: Training Pipeline (Week 7-8)

### 7.1 Create Training Script

Create `scripts/train_g1.py`:

```python
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg
)

class G1LocomotionEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # Adjust for bipedal robot
        self.rewards.feet_air_time.weight = 0.125
        self.rewards.stand_still.weight = -0.5
        self.rewards.orientation.weight = -5.0  # Important for bipeds

        # Curriculum for bipedal learning
        self.curriculum.terrain_levels = 5
```

### 7.2 Training Commands

```bash
# Train standing policy
python scripts/train_g1.py --task=G1-Standing --num_envs=4096

# Train walking policy
python scripts/train_g1.py --task=G1-Walking --num_envs=4096
```

## Phase 8: Testing and Validation (Week 9)

### 8.1 Create Test Suite

Create `tests/test_g1.py`:

```python
def test_g1_standing():
    """Test G1 can maintain standing pose"""
    env = create_g1_env(num_envs=1)
    policy = load_standing_policy()

    obs, _ = env.reset()
    for _ in range(1000):
        actions = policy(obs)
        obs, _, dones, _ = env.step(actions)
        assert not dones[0], "Robot fell"

def test_g1_walking():
    """Test basic forward walking"""
    # Test forward, backward, turning
```

### 8.2 RViz Configuration

Create `rviz/g1.rviz` with G1-specific configurations:

- Joint state visualization (35 joints)
- Center of mass marker
- Foot contact visualization

## Phase 9: Docker Integration (Week 10)

### 9.1 Update Docker Configuration

Modify `docker/Dockerfile`:

```dockerfile
# Add G1-specific dependencies
RUN pip install humanoid-gym

# Copy G1 files
COPY g1/ /workspace/g1/
COPY ckpts/unitree_g1/ /workspace/ckpts/unitree_g1/
```

### 9.2 Launch Scripts

Update `isaac-launch.sh`:

```bash
# Add G1 commands
case "$1" in
    run-g1)
        docker exec -it isaac-sim python isaac_g1_ros2.py
        ;;
    train-g1)
        docker exec -it isaac-sim python scripts/train_g1.py
        ;;
esac
```

## Phase 10: LLM Integration (Optional, Week 11-12)

### 10.1 Cognitive Layer

Create `g1/g1_cognitive.py`:

```python
class G1CognitiveAgent:
    def __init__(self):
        self.llm = self.setup_llm()

    async def process_command(self, text: str):
        """Process natural language commands"""
        # "Walk to the warehouse entrance"
        plan = await self.llm.plan(text)
        return self.convert_to_waypoints(plan)
```

## Implementation Timeline

| Week  | Tasks                         | Deliverables                 |
| ----- | ----------------------------- | ---------------------------- |
| 1     | Model import, directory setup | G1 USD model, file structure |
| 2     | Environment configuration     | G1 env class working         |
| 3     | Control system                | Basic standing controller    |
| 4     | Sensor integration            | LiDAR/camera publishing      |
| 5     | ROS2 bridge                   | Full ROS2 topics for G1      |
| 6     | Main integration              | End-to-end pipeline          |
| 7-8   | RL training                   | Trained walking policy       |
| 9     | Testing                       | Test suite passing           |
| 10    | Docker/deployment             | Containerized solution       |
| 11-12 | LLM integration (optional)    | Natural language control     |

## Key Differences from Go2 Implementation

1. **Degrees of Freedom**: 35 vs 12 - requires larger observation/action spaces
2. **Balance Control**: Critical for bipeds - needs center of mass tracking
3. **Foot Contacts**: Two feet vs four - different contact patterns
4. **Sensor Mounting**: Torso/head mounted vs body mounted
5. **Control Policies**: Standing/walking vs flat/rough terrain
6. **Joint Limits**: More complex kinematic constraints

## Testing Commands

```bash
# Run G1 simulation
python isaac_g1_ros2.py

# Train G1 walking
python scripts/train_g1.py --task=G1-Walking

# Test in Gazebo
ros2 launch g1_gazebo g1_sim.launch.py

# Visualize in RViz
rviz2 -d rviz/g1.rviz
```

This plan maintains your existing architecture while properly adapting for the G1's humanoid characteristics. The modular approach allows you to develop and test each component independently before full integration.
