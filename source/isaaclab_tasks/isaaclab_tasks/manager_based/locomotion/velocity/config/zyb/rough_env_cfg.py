"""
EnvCfg for ZYB Quadruped in Rough Terrain for Velocity Task
"""

from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg
from isaaclab_assets.robots.zyb import B2W_Cfg  # isort: skip
from isaaclab_tasks.manager_based.locomotion.velocity import mdp

@configclass
class ZybQuadrupedArmRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # self.sim.physx.gpu_collision_stack_size = 2**27  # 134MB 量级
        super().__post_init__()
        # ! redefine action space(except wheels)
        self.actions.joint_pos.joint_names = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        ]
        # action scale
        self.actions.joint_pos.scale = 0.2

        # define wheels(4 dementions)
        self.actions.wheel_vel = mdp.JointVelocityActionCfg(
            asset_name="robot",
            joint_names=["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"],
            scale=0.5,            
            use_default_offset=False,  # 轮子一般不需要 default offset
            preserve_order=True,
        )



        self.scene.robot = B2W_Cfg.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base"

        # terrain scaling
        # self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0, 0.05)#(0.05, 0.2)
        # self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0, 0.02)#(0.02, 0.1)
        # self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.02

        # terrain flat
        self.curriculum.terrain_levels = None
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        # events
        self.events.push_robot = None

        self.events.add_base_mass = None
        # self.events.add_base_mass.params["mass_distribution_params"] = (-1.0, 3.0)
        # self.events.add_base_mass.params["asset_cfg"].body_names = "base"

        # self.events.base_external_force_torque.params["asset_cfg"].body_names = "base"  # !
        self.events.base_external_force_torque = None  # ! remove random external force/torque for ZYB
        
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }
        self.events.base_com = None

        # rewards
        # self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_foot"
        # self.rewards.feet_air_time.weight = 0.01
        self.rewards.feet_air_time = None

        # self.rewards.undesired_contacts = None
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = "base|.*_thigh.*|.*_calf.*|.*_hip.*"
        self.rewards.undesired_contacts.weight = -10 #-5.0

        self.rewards.action_rate_l2.weight = -0.02

        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.track_lin_vel_xy_exp.weight = 1.5  #4#1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        self.rewards.dof_acc_l2.weight = -2.5e-7
        self.rewards.flat_orientation_l2.weight = -10#-20#-2 #-0.1#zyb-3  #-1#zyb-2         #-2#zyb-1

        

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = "base"


@configclass
class ZybQuadrupedArmRoughEnvCfg_PLAY(ZybQuadrupedArmRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.num_envs = 1
        self.scene.env_spacing = 3.0
        self.scene.terrain.max_init_terrain_level = None

        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 1
            self.scene.terrain.terrain_generator.num_cols = 1
            self.scene.terrain.terrain_generator.curriculum = False

        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None