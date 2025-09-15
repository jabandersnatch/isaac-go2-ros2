import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
import numpy as np
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg
from isaaclab_assets.robots.unitree import G1_CFG, G1_MINIMAL_CFG
from isaacsim.core.utils.viewports import set_camera_view
from scipy.spatial.transform import Rotation as R

import g1.g1_ctrl as g1_ctrl


@configclass
class G1SimCfg(InteractiveSceneCfg):
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(color=(0.1, 0.1, 0.1), size=(300.0, 300.0)),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0, 0, 1e-4)),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )

    # G1 Robot - using full config for better visibility
    unitree_g1: ArticulationCfg = G1_CFG.replace(prim_path="{ENV_REGEX_NS}/G1")
    # G1 foot contact sensor - using ankle links for humanoid contact detection
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/G1/.*_ankle_roll_link",
        history_length=3,
        track_air_time=True,
    )

    # G1 height scanner - optional for humanoid robots
    height_scanner = None


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    joint_pos = mdp.JointPositionActionCfg(asset_name="unitree_g1", joint_names=[".*"])


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel,
            params={"asset_cfg": SceneEntityCfg(name="unitree_g1")},
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            params={"asset_cfg": SceneEntityCfg(name="unitree_g1")},
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            params={"asset_cfg": SceneEntityCfg(name="unitree_g1")},
            noise=UniformNoiseCfg(n_min=-0.05, n_max=0.05),
        )
        # velocity command
        base_vel_cmd = ObsTerm(func=g1_ctrl.base_vel_cmd)

        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg(name="unitree_g1")},
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg(name="unitree_g1")},
        )
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    base_vel_cmd = mdp.UniformVelocityCommandCfg(
        asset_name="unitree_g1",
        resampling_time_range=(0.0, 0.0),
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.0),
            lin_vel_y=(0.0, 0.0),
            ang_vel_z=(0.0, 0.0),
            heading=(0, 0),
        ),
    )


@configclass
class EventCfg:
    """Configuration for events."""

    pass


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    pass


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    pass


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    pass


@configclass
class G1RSLEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 environment."""

    # scene settings
    scene = G1SimCfg(num_envs=2, env_spacing=2.0)

    # basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()

    # dummy settings
    commands = CommandsCfg()
    rewards = RewardsCfg()
    terminations = TerminationsCfg()
    events = EventCfg()
    curriculum = CurriculumCfg()

    def __post_init__(self):
        # viewer settings
        self.viewer.eye = [-4.0, 0.0, 5.0]
        self.viewer.lookat = [0.0, 0.0, 0.0]

        # step settings
        self.decimation = 8  # step

        # simulation settings
        self.sim.dt = 0.005  # sim step every
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = True
        self.sim.render.antialiasing_mode = None

        # settings for rsl env control
        self.episode_length_s = 20.0  # can be ignored
        self.is_finite_horizon = False
        self.actions.joint_pos.scale = 0.25

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt


def camera_follow(env):
    if env.unwrapped.scene.num_envs == 1:
        robot_position = (
            env.unwrapped.scene["unitree_g1"].data.root_state_w[0, :3].cpu().numpy()
        )
        robot_orientation = (
            env.unwrapped.scene["unitree_g1"].data.root_state_w[0, 3:7].cpu().numpy()
        )
        rotation = R.from_quat(
            [
                robot_orientation[1],
                robot_orientation[2],
                robot_orientation[3],
                robot_orientation[0],
            ]
        )
        yaw = rotation.as_euler("zyx")[0]
        yaw_rotation = R.from_euler("z", yaw).as_matrix()
        set_camera_view(
            yaw_rotation.dot(np.asarray([-4.0, 0.0, 5.0])) + robot_position,
            robot_position,
        )
