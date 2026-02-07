"""
configurations for zyb's quadurped armed robot

"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# ZYB_QUADRUPED_ARM_USD = "/home/zyb/docker/IsaacLab/isaac-sim-zyb/code/assets/robots/quadruped_arm.usd"

ZYB_QUADRUPED_ARM_USD = "/home/zyb/Projects/2026.3_IROS/assets/robots/B2W/b2w.usd"

ZYB_QUADRUPED_ARM_Cfg = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=ZYB_QUADRUPED_ARM_USD,
        activate_contact_sensors=True,

        # rigid body properties
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            # common
            disable_gravity=False,
            retain_accelerations=False,

            # damping
            linear_damping=0.0,
            angular_damping=0.0,

            # limitations
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),

        # articulation properties
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    
    # init states
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.6),
        # B2W
        joint_pos={
            "FL_hip_joint": 0.2, "FL_thigh_joint": 0.8, "FL_calf_joint": -1.6,
            "FR_hip_joint": -0.2, "FR_thigh_joint": 0.8, "FR_calf_joint": -1.6,
            "RL_hip_joint": 0.2, "RL_thigh_joint": 0.8, "RL_calf_joint": -1.6,
            "RR_hip_joint": -0.2, "RR_thigh_joint": 0.8, "RR_calf_joint": -1.6,
            "FL_foot_joint": 0.0,
            "FR_foot_joint": 0.0,
            "RL_foot_joint": 0.0,
            "RR_foot_joint": 0.0,
        },


        # joint_pos={
        #     "FL_hip_joint": 0.2, "FL_thigh_joint": 0.8, "FL_calf_joint": -1.6,
        #     "FR_hip_joint": -0.2, "FR_thigh_joint": 0.8, "FR_calf_joint": -1.6,
        #     "RL_hip_joint": 0.2, "RL_thigh_joint": 0.8, "RL_calf_joint": -1.6,
        #     "RR_hip_joint": -0.2, "RR_thigh_joint": 0.8, "RR_calf_joint": -1.6,

        #     "FL_foot_wheel_joint": 0.0,
        #     "FR_foot_wheel_joint": 0.0,
        #     "RL_foot_wheel_joint": 0.0,
        #     "RR_foot_wheel_joint": 0.0,

        #     "joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0,
        #     "joint5": 0.0, "joint6": 0.0,

        #     "joint7": 0.0,
        #     "joint8": 0.0,
        # },

        joint_vel={".*": 0.0},
    ),

    actuators={
        # B2W
        "legs": DCMotorCfg(
            joint_names_expr=[
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",

                # "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint",
            ],
            # effort_limit=23.5,
			saturation_effort=50,#23.5,  # 饱和力矩
			velocity_limit=30.0,
            stiffness=80,  #50.0,  # K_p
            damping=2,  #1.0,  # K_d
            friction=0.0,
        ),




        # 4足->12 joints
        # "legs": DCMotorCfg(
        #     joint_names_expr=[
        #         "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        #         "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        #         "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        #         "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",

        #         "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint",
        #     ],
        #     # effort_limit=23.5,
		# 	saturation_effort=50,#23.5,  # 饱和力矩
		# 	velocity_limit=30.0,
        #     stiffness=80,  #50.0,  # K_p
        #     damping=2,  #1.0,  # K_d
        #     friction=0.0,
        # ),

        # # foot_wheels
        # "foot_wheels": ImplicitActuatorCfg(
        #     joint_names_expr=[
        #         "FL_foot_wheel_joint",
        #         "FR_foot_wheel_joint",
        #         "RL_foot_wheel_joint",
        #         "RR_foot_wheel_joint",
        #     ],
        #     stiffness=0.0,
        #     damping=0.5,
        # ),



        # # 6 revolutes
        # "arm_rev": ImplicitActuatorCfg(
        #     joint_names_expr=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        #     stiffness=5.0,
        #     damping=0.5,
        # ),

        # # prismatic
        # "arm_prismatic": ImplicitActuatorCfg(
        #     joint_names_expr=["joint7", "joint8"],
        #     stiffness=5.0,
        #     damping=0.5,
        # ),
    },
)
