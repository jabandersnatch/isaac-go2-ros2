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
    description="G1 Humanoid robot simulation with ROS2 integration."
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
import g1.g1_ctrl as g1_ctrl
import g1.g1_sensors as g1_sensors
import ros2.g1_ros2_bridge as g1_ros2_bridge
from g1.g1_env import G1RSLEnvCfg, camera_follow

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")


@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):

    # G1 Environment setup
    g1_env_cfg = G1RSLEnvCfg()
    g1_env_cfg.scene.num_envs = cfg.num_envs
    g1_env_cfg.decimation = math.ceil(1.0 / g1_env_cfg.sim.dt / cfg.freq)
    g1_env_cfg.sim.render_interval = g1_env_cfg.decimation
    g1_ctrl.init_base_vel_cmd(cfg.num_envs)
    env, controller = g1_ctrl.get_g1_basic_controller(g1_env_cfg)

    # Simulation environment
    if cfg.env_name == "obstacle-dense":
        sim_env.create_obstacle_dense_env()  # obstacles dense
    elif cfg.env_name == "obstacle-medium":
        sim_env.create_obstacle_medium_env()  # obstacles medium
    elif cfg.env_name == "obstacle-sparse":
        sim_env.create_obstacle_sparse_env()  # obstacles sparse
    elif cfg.env_name == "warehouse":
        sim_env.create_warehouse_env()  # warehouse
    elif cfg.env_name == "warehouse-forklifts":
        sim_env.create_warehouse_forklifts_env()  # warehouse forklifts
    elif cfg.env_name == "warehouse-shelves":
        sim_env.create_warehouse_shelves_env()  # warehouse shelves
    elif cfg.env_name == "full-warehouse":
        sim_env.create_full_warehouse_env()  # full warehouse

    # Sensor setup
    sm = g1_sensors.SensorManager(cfg.num_envs)
    lidar_annotators = []
    cameras = []

    # Only add sensors if they are enabled in config
    if cfg.sensor.enable_lidar:
        lidar_annotators = sm.add_rtx_lidar()
    if cfg.sensor.enable_camera:
        cameras = sm.add_camera(cfg.freq)

    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(),
        g1_ctrl.sub_keyboard_event,
    )

    # ROS2 Bridge
    rclpy.init()
    dm = g1_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)

    # Run simulation
    sim_step_dt = float(g1_env_cfg.sim.dt * g1_env_cfg.decimation)
    obs, _ = env.reset()

    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():
            # control joints
            actions = controller.get_actions(obs)

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
