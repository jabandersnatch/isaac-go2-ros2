import argparse
import math
import os
import time

import hydra
import rclpy
import torch
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="Unified Unitree robot simulation with ROS2 integration."
)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import carb
import omni
import torch

import env.sim_env as sim_env

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")


@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):

    # Determine robot type from config
    robot_type = cfg.get("robot_type", "go2")  # Default to go2

    if robot_type == "g1":
        print("Launching G1 Humanoid Robot Simulation...")
        import g1.g1_ctrl as robot_ctrl
        import g1.g1_sensors as sensors
        import ros2.g1_ros2_bridge as ros2_bridge  # Use working Go2 bridge
        from g1.g1_env import G1RSLEnvCfg, camera_follow

        # G1 Environment setup
        env_cfg = G1RSLEnvCfg()
        env_cfg.scene.num_envs = cfg.num_envs
        env_cfg.decimation = math.ceil(1.0 / env_cfg.sim.dt / cfg.freq)
        env_cfg.sim.render_interval = env_cfg.decimation
        robot_ctrl.init_base_vel_cmd(cfg.num_envs)

        env, policy = robot_ctrl.get_g1_basic_controller(env_cfg)

    else:  # Default to Go2
        print("Launching Go2 Quadruped Robot Simulation...")
        import go2.go2_ctrl as robot_ctrl
        import go2.go2_sensors as sensors
        import ros2.go2_ros2_bridge as ros2_bridge
        from go2.go2_env import Go2RSLEnvCfg, camera_follow

        # Go2 Environment setup
        env_cfg = Go2RSLEnvCfg()
        env_cfg.scene.num_envs = cfg.num_envs
        env_cfg.decimation = math.ceil(1.0 / env_cfg.sim.dt / cfg.freq)
        env_cfg.sim.render_interval = env_cfg.decimation
        robot_ctrl.init_base_vel_cmd(cfg.num_envs)

        # Use RL policy for Go2
        env, policy = robot_ctrl.get_rsl_rough_policy(env_cfg)

    # Simulation environment setup (same for both robots)
    if cfg.env_name == "obstacle-dense":
        sim_env.create_obstacle_dense_env()
    elif cfg.env_name == "obstacle-medium":
        sim_env.create_obstacle_medium_env()
    elif cfg.env_name == "obstacle-sparse":
        sim_env.create_obstacle_sparse_env()
    elif cfg.env_name == "warehouse":
        sim_env.create_warehouse_env()
    elif cfg.env_name == "warehouse-forklifts":
        sim_env.create_warehouse_forklifts_env()
    elif cfg.env_name == "warehouse-shelves":
        sim_env.create_warehouse_shelves_env()
    elif cfg.env_name == "full-warehouse":
        sim_env.create_full_warehouse_env()

    # Sensor setup
    sm = sensors.SensorManager(cfg.num_envs)
    lidar_annotators = []
    cameras = []

    if cfg.sensor.enable_lidar:
        lidar_annotators = sm.add_rtx_lidar()
    if cfg.sensor.enable_camera:
        cameras = sm.add_camera(cfg.freq)

    # Keyboard control (only if GUI is available)
    app_window = omni.appwindow.get_default_app_window()
    if app_window is not None:
        system_input = carb.input.acquire_input_interface()
        system_input.subscribe_to_keyboard_events(
            app_window.get_keyboard(), robot_ctrl.sub_keyboard_event
        )

    # ROS2 Bridge
    print("[DEBUG] Initializing ROS2 bridge")
    rclpy.init()
    dm = ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)
    print("[DEBUG] ROS2 bridge initialized successfully")

    # Run simulation
    sim_step_dt = float(env_cfg.sim.dt * env_cfg.decimation)

    if robot_type == "g1":
        # G1 basic controller loop with proper observation flow
        obs, _ = env.reset()
        while simulation_app.is_running():
            start_time = time.time()
            with torch.inference_mode():
                # control joints
                actions = policy.get_actions(obs)

                # step the environment
                step_result = env.step(actions)
                obs = step_result[0]

                # ROS2 data publishing
                dm.pub_ros2_data()
                rclpy.spin_once(dm)

                # Camera follow
                if cfg.camera_follow:
                    camera_follow(env)

                # limit loop time
                elapsed_time = time.time() - start_time
                if elapsed_time < sim_step_dt:
                    sleep_duration = sim_step_dt - elapsed_time
                    time.sleep(sleep_duration)
            actual_loop_time = time.time() - start_time
            rtf = min(1.0, sim_step_dt / elapsed_time)
            print(
                f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}",
                end="",
                flush=True,
            )
    else:
        # Go2 RL policy loop
        obs, _ = env.reset()
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
                if cfg.camera_follow:
                    camera_follow(env)

                # limit loop time
                elapsed_time = time.time() - start_time
                if elapsed_time < sim_step_dt:
                    sleep_duration = sim_step_dt - elapsed_time
                    time.sleep(sleep_duration)
            actual_loop_time = time.time() - start_time
            rtf = min(1.0, sim_step_dt / elapsed_time)
            print(
                f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}",
                end="",
                flush=True,
            )

    dm.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    run_simulator()

