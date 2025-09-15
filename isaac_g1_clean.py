import os
import hydra
import rclpy
import torch
import time
import math
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="G1 Humanoid robot simulation with ROS2 integration - CLEAN VERSION.")

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
import g1.g1_ctrl_simple as g1_ctrl  # Use simple controller that works
import ros2.go2_ros2_bridge as ros2_bridge  # Use Go2's working ROS2 bridge

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    print("[CLEAN] Starting clean G1 simulation using Go2 patterns")
    
    # G1 Environment setup (like Go2)
    g1_env_cfg = G1RSLEnvCfg()
    g1_env_cfg.scene.num_envs = cfg.num_envs
    g1_env_cfg.decimation = math.ceil(1./g1_env_cfg.sim.dt/cfg.freq)
    g1_env_cfg.sim.render_interval = g1_env_cfg.decimation
    g1_ctrl.init_base_vel_cmd(cfg.num_envs)
    
    # Create environment and controller (like Go2)
    env, policy = g1_ctrl.get_g1_simple_controller(g1_env_cfg)
    print("[CLEAN] G1 environment and controller created")

    # Simulation environment (like Go2)
    if (cfg.env_name == "obstacle-dense"):
        sim_env.create_obstacle_dense_env()
    elif (cfg.env_name == "warehouse"):
        sim_env.create_warehouse_env()
    print(f"[CLEAN] Environment: {cfg.env_name}")

    # Sensor setup (like Go2)
    sm = g1_sensors.SensorManager(cfg.num_envs)
    lidar_annotators = []
    cameras = []
    if cfg.sensor.enable_camera:
        cameras = sm.add_camera(cfg.freq)
    print("[CLEAN] Sensors setup")

    # Keyboard control (like Go2)
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), g1_ctrl.sub_keyboard_event)
    print("[CLEAN] Keyboard control setup")

    # ROS2 Bridge (use Go2's working bridge)
    rclpy.init()
    dm = ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)
    print("[CLEAN] ROS2 bridge initialized")

    # CLEAN MAIN LOOP - EXACTLY LIKE GO2
    print("[CLEAN] Starting simulation loop")
    sim_step_dt = float(g1_env_cfg.sim.dt * g1_env_cfg.decimation)
    
    obs, _ = env.reset()
    print("[CLEAN] Environment reset, starting main loop")
    
    # EXACT GO2 PATTERN
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():            
            # control joints
            actions = policy(obs)

            # step the environment
            step_result = env.step(actions)
            obs = step_result[0]

            # ROS2 data
            dm.pub_ros2_data()
            rclpy.spin_once(dm)

            # Camera follow
            if (cfg.camera_follow):
                camera_follow(env)

            # limit loop time
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
                
            actual_loop_time = time.time() - start_time
            rtf = min(1.0, sim_step_dt/elapsed_time)
            print(f"\rStep time: {actual_loop_time*1000:.2f}ms, RTF: {rtf:.2f}", end='', flush=True)
    
    print("[CLEAN] Shutting down")
    dm.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()