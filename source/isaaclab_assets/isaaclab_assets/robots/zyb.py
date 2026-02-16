"""
configurations for zyb's quadurped armed robot

"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg, IdealPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

ZYB_QUADRUPED_ARM_USD = "/home/zyb/docker/IsaacLab/isaac-sim-zyb/code/assets/robots/quadruped_arm.usd"

B2W_USD = "/home/users/wenzong.ma/zyb/assets/robots/b2w.usd"

B2W_FIXED_USD = "/home/users/wenzong.ma/zyb/assets/robots/B2W_fixed_wheel.usd"

B2W_Cfg = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=B2W_USD,
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
        pos=(0.0, 0.0, 0.58),
        # B2
        joint_pos={
            ".*R_hip_joint": -0.1,
            ".*L_hip_joint": 0.1,
            "F[L,R]_thigh_joint": 0.8,
            "R[L,R]_thigh_joint": 1.0,
            ".*_calf_joint": -1.5,

            "FL_foot_joint": 0.0,
            "FR_foot_joint": 0.0,
            "RL_foot_joint": 0.0,
            "RR_foot_joint": 0.0,
        },

        joint_vel={".*": 0.0},
    ),

    actuators={
        "M107-24-2": IdealPDActuatorCfg(
            joint_names_expr=[".*_hip_.*", ".*_thigh_.*"],
            effort_limit=200,
            velocity_limit=23,
            stiffness=160,
            damping=5,
            friction=0.01,
        ),
        "2": IdealPDActuatorCfg(
            joint_names_expr=[".*_calf_.*"],
            effort_limit=320,
            velocity_limit=14,
            stiffness=160.0,
            damping=5.0,
            friction=0.01,
        ),
        "wheels": DCMotorCfg(
            joint_names_expr=[".*_foot_joint"],
            effort_limit=20.0,
            velocity_limit=50.0,     # rad/s
            stiffness=0.0,           # 速度控制时通常不靠 stiffness
            damping=2.0,             # ✅ 从 0.5~5 起步，别上来 50000（容易数值僵硬/不稳定）
            friction=0.2,            # joint friction（关节阻尼/摩擦感），不是轮胎抓地
            saturation_effort=20.0,  # DCMotorCfg 需要这个峰值扭矩参数
        ),
    },
)


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
        pos=(0.0, 0.0, 0.58),

        joint_pos={
            ".*R_hip_joint": -0.1,
            ".*L_hip_joint": 0.1,
            "F[L,R]_thigh_joint": 0.8,
            "R[L,R]_thigh_joint": 1.0,
            ".*_calf_joint": -1.5,

            "FL_foot_joint": 0.0,
            "FR_foot_joint": 0.0,
            "RL_foot_joint": 0.0,
            "RR_foot_joint": 0.0,
        },

        joint_vel={".*": 0.0},
    ),

    actuators={
        "M107-24-2": IdealPDActuatorCfg(
            joint_names_expr=[".*_hip_.*", ".*_thigh_.*"],
            effort_limit=200,
            velocity_limit=23,
            stiffness=160,
            damping=5,
            friction=0.01,
        ),
        "2": IdealPDActuatorCfg(
            joint_names_expr=[".*_calf_.*"],
            effort_limit=320,
            velocity_limit=14,
            stiffness=160.0,
            damping=5.0,
            friction=0.01,
        ),
        "wheels": IdealPDActuatorCfg(
            joint_names_expr=[".*_foot_joint"],
            effort_limit=20.0,
            velocity_limit=50.0,
            stiffness=0.0,
            damping=0.2,
            friction=0.01,
        ),
        "arms": IdealPDActuatorCfg(
            joint_names_expr=[".*_arm_.*"],
            effort_limit=100.0,
            velocity_limit=10.0,
            stiffness=160.0,
            damping=5.0,
            friction=0.01,
        ),
    },
)