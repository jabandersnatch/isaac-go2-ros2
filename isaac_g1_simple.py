import os
import hydra
import rclpy
import torch
import time
import math
import argparse
from isaaclab.app import AppLauncher
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# add argparse arguments
parser = argparse.ArgumentParser(description="G1 Humanoid robot simulation with ROS2 integration - SIMPLIFIED.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

from g1.g1_env import G1RSLEnvCfg, camera_follow
import env.sim_env as sim_env
import g1.g1_sensors as g1_sensors
import omni
import carb
import g1.g1_ctrl_simple as g1_ctrl  # Use simple controller
import ros2.g1_ros2_bridge as g1_ros2_bridge

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    logger.info("[MAIN] Starting G1 Simple Simulation (following Go2 pattern)")
    
    try:
        # G1 Environment setup - exactly like Go2
        g1_env_cfg = G1RSLEnvCfg()
        g1_env_cfg.scene.num_envs = cfg.num_envs
        g1_env_cfg.decimation = math.ceil(1./g1_env_cfg.sim.dt/cfg.freq)
        g1_env_cfg.sim.render_interval = g1_env_cfg.decimation
        logger.info(f"[MAIN] Environment config complete")
        
        g1_ctrl.init_base_vel_cmd(cfg.num_envs)
        
        # Create environment and simple controller
        env, policy = g1_ctrl.get_g1_simple_controller(g1_env_cfg)
        logger.info("[MAIN] G1 environment and simple controller created")
        
    except Exception as e:
        logger.error(f"[MAIN] Failed to setup G1: {str(e)}")
        simulation_app.close()
        return

    try:
        # Simple environment setup - like Go2
        if (cfg.env_name == "default"):
            logger.info("[MAIN] Using default environment")
        else:
            logger.info(f"[MAIN] Using environment: {cfg.env_name}")
        
    except Exception as e:
        logger.error(f"[MAIN] Environment setup warning: {str(e)}")

    try:
        # Simplified sensor setup
        sm = g1_sensors.SensorManager(cfg.num_envs)
        lidar_annotators = []
        cameras = []
        
        if cfg.sensor.enable_camera:
            cameras = sm.add_camera(cfg.freq)
            logger.info("[MAIN] Camera sensors enabled")
        
    except Exception as e:
        logger.error(f"[MAIN] Sensor setup warning: {str(e)}")
        lidar_annotators = []
        cameras = []

    try:
        # Keyboard control - like Go2
        system_input = carb.input.acquire_input_interface()
        system_input.subscribe_to_keyboard_events(
            omni.appwindow.get_default_app_window().get_keyboard(), g1_ctrl.sub_keyboard_event)
        logger.info("[MAIN] Keyboard control setup")
        
    except Exception as e:
        logger.error(f"[MAIN] Keyboard setup warning: {str(e)}")
    
    try:
        # ROS2 Bridge
        rclpy.init()
        dm = g1_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)
        logger.info("[MAIN] ROS2 bridge initialized")
        
    except Exception as e:
        logger.error(f"[MAIN] ROS2 setup failed: {str(e)}")
        simulation_app.close()
        return

    # SIMPLIFIED MAIN LOOP - EXACTLY LIKE GO2
    logger.info("[MAIN] Starting simplified simulation loop")
    sim_step_dt = float(g1_env_cfg.sim.dt * g1_env_cfg.decimation)
    
    obs, _ = env.reset()
    logger.info("[MAIN] Environment reset, starting main loop")
    
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():            
            # Control joints - EXACTLY LIKE GO2
            actions = policy(obs)

            # Step the environment - EXACTLY LIKE GO2
            step_result = env.step(actions)
            obs = step_result[0]

            # ROS2 data - EXACTLY LIKE GO2
            dm.pub_ros2_data()
            rclpy.spin_once(dm)

            # Camera follow - EXACTLY LIKE GO2
            if (cfg.camera_follow):
                camera_follow(env)

            # Limit loop time - EXACTLY LIKE GO2
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, sim_step_dt/elapsed_time)
        print(f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}", end='', flush=True)
    
    logger.info("[MAIN] Shutting down")
    try:
        dm.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        logger.error(f"[MAIN] Shutdown warning: {e}")
    
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()