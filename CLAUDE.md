# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Isaac Sim robotics simulation project featuring Unitree robots with ROS2 integration. It combines NVIDIA Isaac Sim 4.5, Isaac Lab 2.1, and ROS2 Humble to provide complete robot simulation environments with sensor data publishing, keyboard teleoperation, and Docker containerization.

**Supported Robots:**
- **Unitree Go2**: Quadruped robot with trained RL policies for locomotion control
- **Unitree G1**: Humanoid robot with basic joint position control (locomotion policies in development)

## Architecture

The project follows a layered architecture:

- **Simulation Layer**: Built on Isaac Sim/Isaac Lab for physics simulation and rendering
- **Robot Layer**: Unitree robots (Go2 quadruped with RL policies, G1 humanoid with basic control)
- **Sensor Layer**: LiDAR, RGB/Depth cameras, semantic segmentation, contact sensors
- **ROS2 Bridge Layer**: Publishes robot state, sensor data, and handles velocity commands
- **Environment Layer**: Multiple configurable simulation environments (warehouses, obstacle fields)
- **Docker Layer**: Containerized deployment with X11 forwarding for GUI support

## Common Development Commands

### Using Docker (Recommended)
```bash
# Build the ROS workspace
./isaac-launch.sh build

# Run complete simulation with GUI and ROS2 bridge
./isaac-launch.sh run

# Launch Isaac Sim GUI only (for development/testing)
./isaac-launch.sh sim

# Enter container for interactive development
./isaac-launch.sh enter

# Run custom command in container
./isaac-launch.sh --command "python isaac_unified_ros2.py"

# Stop the container
./isaac-launch.sh close
```

### Native Development (if not using Docker)
```bash
# Activate Isaac Lab conda environment
conda activate isaaclab

# Run the main simulation (Go2)
python isaac_go2_ros2.py

# Run G1 simulation
python isaac_g1_ros2.py

# Run unified simulation (robot type from config)
python isaac_unified_ros2.py

# Visualize ROS2 data in RViz2
rviz2 -d rviz/go2.rviz
```

### Testing and Validation
```bash
# Check ROS2 topics are publishing
ros2 topic list

# Monitor odometry data
ros2 topic echo /unitree_go2/odom

# Send velocity commands for testing
ros2 topic pub /unitree_go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```

## Configuration

### Simulation Settings
Edit `cfg/sim.yaml` to configure:
- **Robot Selection** (`robot_type`): `go2` (quadruped) or `g1` (humanoid)
- Number of robot instances (`num_envs`)
- Simulation frequency (`freq`)
- Environment type (`env_name`: warehouse, obstacle-dense, etc.)
- Sensor enablement (LiDAR, camera types)
- Display settings (resolution, anti-aliasing)

### Environment Variables
Key Docker environment variables in `docker/.env`:
- `ROS_DOMAIN_ID`: ROS2 network domain (default: 100)
- `AUTO_BUILD`: Automatically build ROS workspace on container start
- `AUTO_RUN`: Automatically run main simulation on container start

## Key Components

### Main Entry Points
- `isaac_go2_ros2.py`: Go2 quadruped simulation launcher
- `isaac_g1_ros2.py`: G1 humanoid simulation launcher  
- `isaac_unified_ros2.py`: Unified launcher (robot type from config)

### Robot Control
**Go2 (Quadruped):**
- `go2/go2_ctrl.py`: Keyboard input handling and RL policy execution
- `go2/go2_env.py`: Robot environment configuration and scene setup
- `go2/go2_sensors.py`: Sensor management (LiDAR, cameras)

**G1 (Humanoid):**
- `g1/g1_ctrl.py`: Keyboard input handling and basic joint control
- `g1/g1_env.py`: Robot environment configuration and scene setup
- `g1/g1_sensors.py`: Sensor management (cameras, optional LiDAR)

### ROS2 Integration
- `ros2/go2_ros2_bridge.py`: Go2 ROS2 data publishing and command subscription
- `ros2/g1_ros2_bridge.py`: G1 ROS2 data publishing with joint states

### Simulation Environments
- `env/sim_env.py`: Various USD environment configurations (warehouse, obstacles)

### Trained Models
- `ckpts/unitree_go2/`: Pre-trained RL policies for flat and rough terrain locomotion

## ROS2 Topics

### Go2 Quadruped Topics
**Published Topics**:
- `/unitree_go2/odom`: Robot odometry (nav_msgs/Odometry)
- `/unitree_go2/pose`: Robot pose (geometry_msgs/PoseStamped)  
- `/unitree_go2/lidar/point_cloud`: LiDAR data (sensor_msgs/PointCloud2)
- `/unitree_go2/front_cam/color_image`: RGB camera (sensor_msgs/Image)
- `/unitree_go2/front_cam/depth_image`: Depth camera (sensor_msgs/Image)
- `/unitree_go2/front_cam/semantic_segmentation_image`: Semantic segmentation (sensor_msgs/Image)
- `/unitree_go2/front_cam/info`: Camera intrinsics (sensor_msgs/CameraInfo)

**Subscribed Topics**:
- `/unitree_go2/cmd_vel`: Velocity commands (geometry_msgs/Twist)

### G1 Humanoid Topics
**Published Topics**:
- `/unitree_g1/odom`: Robot odometry (nav_msgs/Odometry)
- `/unitree_g1/pose`: Robot pose (geometry_msgs/PoseStamped)
- `/unitree_g1/joint_states`: Joint positions and velocities (sensor_msgs/JointState)
- `/unitree_g1/lidar/point_cloud`: LiDAR data (sensor_msgs/PointCloud2) [optional]
- `/unitree_g1/front_cam/color_image`: RGB camera (sensor_msgs/Image)
- `/unitree_g1/front_cam/depth_image`: Depth camera (sensor_msgs/Image)
- `/unitree_g1/front_cam/semantic_segmentation_image`: Semantic segmentation (sensor_msgs/Image)
- `/unitree_g1/front_cam/info`: Camera intrinsics (sensor_msgs/CameraInfo)

**Subscribed Topics**:
- `/unitree_g1/cmd_vel`: Velocity commands (geometry_msgs/Twist)

## Keyboard Controls (when simulation window is focused)
- `W`: Forward movement
- `S`: Backward movement  
- `A`: Left strafe
- `D`: Right strafe
- `Z`: Left turn
- `C`: Right turn

## Development Notes

### Adding New Environments
1. Create USD scene in `env/sim_env.py`
2. Add environment selection logic in `isaac_go2_ros2.py` 
3. Update `cfg/sim.yaml` with new environment name

### Modifying Robot Configuration
- Robot spawn configuration: `go2/go2_env.py` (Go2SimCfg class)
- Sensor setup: `go2/go2_sensors.py` 
- Control parameters: `go2/go2_ctrl_cfg.py`

### Multi-Robot Support
The system supports multiple robots by setting `num_envs > 1` in config. Topics are namespaced as `/unitree_go2_{i}/...` for robot index i.

### Docker Build Process
The Dockerfile installs Isaac Sim 4.5, ROS2 Humble, Isaac Lab 2.1, and conda environment in sequential stages for optimal caching.

## Detailed Architecture Analysis

This section provides in-depth technical documentation of how the Isaac Go2 ROS2 system works, designed to facilitate replication with other Unitree robots (A1, A2, B1, etc.).

### Main Entry Point Architecture (`isaac_go2_ros2.py`)

The `isaac_go2_ros2.py` file serves as the orchestrator for the entire simulation system. It follows a structured initialization and execution pattern:

#### Initialization Flow:
1. **Argument Processing & App Launch**: Uses Isaac Lab's `AppLauncher` to handle command-line arguments and initialize the Omniverse application
2. **Configuration Loading**: Uses Hydra to load simulation configuration from `cfg/sim.yaml`
3. **Environment Setup**: Initializes robot environment, policy loading, scene creation, sensors, and ROS2 bridge
4. **Main Loop**: Runs the physics simulation with real-time factor control

#### Key Components Instantiation:
```python
# Go2 Environment setup (isaac_go2_ros2.py:39-45)
go2_env_cfg = Go2RSLEnvCfg()
go2_env_cfg.scene.num_envs = cfg.num_envs
go2_env_cfg.decimation = math.ceil(1./go2_env_cfg.sim.dt/cfg.freq)
env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)

# Sensor setup (isaac_go2_ros2.py:64-66)
sm = go2_sensors.SensorManager(cfg.num_envs)
lidar_annotators = sm.add_rtx_lidar()
cameras = sm.add_camera(cfg.freq)

# ROS2 Bridge (isaac_go2_ros2.py:74-75)
rclpy.init()
dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)
```

### Environment System (`env/` Directory)

The environment system provides modular scene creation capabilities:

#### Scene Types and Configuration:
- **Procedural Obstacles** (`create_obstacle_*_env()`): Generates terrain with randomly placed obstacles
  - Uses custom `HfUniformDiscreteObstaclesTerrainCfg` configuration
  - Configurable obstacle density (sparse: 100, medium: 200, dense: 400 obstacles)
  - Maintains clear spawn area at origin with `avoid_positions=[[0, 0]]`

- **Warehouse Scenes** (`create_warehouse_*_env()`): Loads pre-built USD assets from Isaac Sim
  - References assets via `nucleus_utils.get_assets_root_path()`
  - Multiple variants: simple, with forklifts, with shelves, full warehouse

#### Custom Terrain Generation (`env/terrain.py`):
The `uniform_discrete_obstacles_terrain` function implements sophisticated obstacle placement:
- **Collision Avoidance**: Uses `is_good_position()` to ensure minimum distance between obstacles
- **Timeout Protection**: Prevents infinite loops in obstacle placement with 0.2s timeout
- **Central Platform**: Maintains obstacle-free area around robot spawn point
- **Heightfield Generation**: Converts obstacle specifications to Isaac Lab heightfield format

### Robot Configuration System (`go2/` Directory)

#### Robot Environment Configuration (`go2/go2_env.py`):
The `Go2RSLEnvCfg` class defines the complete robot simulation environment:

##### Scene Configuration (`Go2SimCfg`):
- **Ground Plane**: 300x300m ground with physics material
- **Lighting**: Distant light (3000.0 intensity) + dome light (500.0 intensity) for realistic rendering
- **Robot Definition**: References `UNITREE_GO2_CFG` from Isaac Lab assets with `{ENV_REGEX_NS}/Go2` pattern for multi-environment support

##### Sensor Integration:
- **Contact Sensors**: Track foot contact states with 3-frame history and air time
- **Height Scanner**: Ray-casting grid (1.6x1.0m, 0.1m resolution) for terrain perception

##### Observation System (`ObservationsCfg`):
Critical for RL policy operation, provides:
- **Robot State**: Base linear/angular velocity, projected gravity, joint positions/velocities
- **Command Interface**: Velocity commands from keyboard/ROS2 input
- **Terrain Perception**: Height scan data for rough terrain navigation
- **Action History**: Previous actions for temporal coherence

#### Control System (`go2/go2_ctrl.py`):
Implements dual-mode control architecture:

##### Policy Loading:
- **Flat Terrain Policy**: Optimized for smooth surfaces (`get_rsl_flat_policy()`)
- **Rough Terrain Policy**: Handles complex terrain (`get_rsl_rough_policy()`)
- **Checkpoint Management**: Loads pre-trained models from `ckpts/unitree_go2/` directory

##### Command Processing:
- **Global Tensor**: `base_vel_cmd_input` stores velocity commands for all environments
- **Keyboard Mapping**: WASD for translation, ZC for rotation, IJKL/M> for second robot
- **ROS2 Integration**: Commands published to `/cmd_vel` update the same tensor

#### Sensor Management (`go2/go2_sensors.py`):
Provides unified sensor initialization:

##### LiDAR Configuration:
- **RTX LiDAR**: Uses Hesai XT32_SD10 configuration (hardware-accurate)
- **Positioning**: Mounted at (0.2, 0, 0.2) offset from robot base
- **Data Pipeline**: Uses Replicator annotations for efficient data extraction

##### Camera Configuration:
- **RGB Camera**: 640x480 resolution, 1.5 focal length
- **Positioning**: Front-mounted at (0.4, 0, 0.2) offset
- **Multi-Modal**: Supports RGB, depth, and semantic segmentation

### ROS2 Bridge Architecture (`ros2/go2_ros2_bridge.py`)

The ROS2 bridge implements a comprehensive data publishing and subscription system:

#### Core Architecture:
- **Multi-Robot Support**: Dynamically creates topics for N robots (`/unitree_go2_{i}/...`)
- **Time Synchronization**: Publishes simulation time to `/clock` topic
- **Transform Management**: Static transforms for sensor frames, dynamic transforms for robot pose

#### Data Publishing Pipeline:

##### Odometry Publishing (`publish_odom()`):
- **Source**: Robot's `root_state_w` (world position/orientation) and `root_lin_vel_b`/`root_ang_vel_b` (body velocities)
- **Frame Hierarchy**: `map` → `base_link` transform chain
- **Frequency**: 50Hz via wall-time control

##### Sensor Data Publishing:
- **LiDAR**: Point cloud data at 15Hz from RTX LiDAR annotations
- **RGB/Depth Images**: Published via Isaac Sim's synthetic data writers
- **Semantic Segmentation**: Custom colorization pipeline for visualization

##### Transform Broadcasting:
- **Static Transforms**: Sensor to base_link relationships defined at initialization
- **Dynamic Transforms**: Real-time robot pose in world coordinates

#### Command Subscription:
- **Velocity Commands**: `/cmd_vel` messages directly update global control tensor
- **Bidirectional Control**: Supports both keyboard and ROS2 velocity commands simultaneously

### Multi-Environment Support

The system is designed for scalable multi-robot simulation:

#### Environment Indexing:
- **Prim Path Pattern**: `/World/envs/env_{env_idx}/Go2` for Isaac Sim scene graph
- **Topic Namespacing**: `/unitree_go2_{env_idx}/...` for ROS2 topics when `num_envs > 1`
- **Sensor Isolation**: Each environment has independent sensor instances

#### Resource Management:
- **GPU Memory**: Shared physics simulation with per-robot state tensors
- **ROS2 Publishers**: Dynamically allocated per environment during initialization

## Adaptation Guide for Other Unitree Robots

To replicate this architecture for other Unitree robots (A1, A2, B1, etc.), follow these systematic modifications:

### 1. Robot Asset Replacement
Replace `UNITREE_GO2_CFG` imports and references:
```python
# In go2/go2_env.py, replace line 2:
from isaaclab_assets.robots.unitree import UNITREE_A1_CFG  # or A2, B1, etc.

# Update line 47:
unitree_a1: ArticulationCfg = UNITREE_A1_CFG.replace(prim_path="{ENV_REGEX_NS}/A1")
```

### 2. Configuration Adaptation
Update policy configurations in `go2/go2_ctrl_cfg.py`:
- **Checkpoint Paths**: Point to robot-specific trained models
- **Network Architecture**: Adjust hidden layers if observation space differs
- **Hyperparameters**: Tune learning rates, clip parameters for robot characteristics

### 3. Sensor Configuration Updates
Modify sensor mounting positions in `go2/go2_sensors.py`:
- **LiDAR Position**: Adjust mounting offset based on robot geometry
- **Camera Position**: Update position for optimal field of view
- **Contact Sensors**: Modify foot sensor paths to match robot's URDF structure

### 4. Control Interface Adaptation
Update control parameters:
- **Velocity Limits**: Adjust `lin_vel` and `ang_vel` in keyboard handler for robot capabilities
- **Action Scaling**: Modify `actions.joint_pos.scale` for robot's joint ranges
- **Joint Names**: Update joint patterns in `ActionsCfg` to match robot's kinematic structure

### 5. ROS2 Topic Renaming
Update topic namespaces throughout `ros2/go2_ros2_bridge.py`:
- Replace `"unitree_go2"` with `"unitree_a1"` (or appropriate robot name)
- Update frame IDs in transform broadcasts
- Modify checkpoint directory references

### 6. Environment File Updates
Rename and update files:
- Copy `go2/` directory to `a1/` (or appropriate robot name)
- Update all import statements to reference new robot modules
- Modify main entry point to import from new robot-specific modules

### 7. Multi-Robot Heterogeneous Support
For mixed robot deployments:
- **Robot Type Configuration**: Add robot type to simulation config
- **Conditional Loading**: Load appropriate robot assets based on environment index
- **Type-Specific Topics**: Include robot type in topic naming convention

This architecture provides a robust foundation for multi-robot simulation with different Unitree platforms, enabling complex research scenarios involving heterogeneous robot teams.