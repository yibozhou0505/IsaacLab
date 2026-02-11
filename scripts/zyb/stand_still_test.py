import argparse
import torch

from isaaclab.app import AppLauncher
from isaaclab.envs import ManagerBasedRLEnv

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str, required=True)
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument("--steps", type=int, default=2000)  # 2000步够看
    args = parser.parse_args()

    # 启动 Isaac Sim（headless 也行）
    app_launcher = AppLauncher(headless=False)  # 远程不想开UI就改 True
    sim_app = app_launcher.app

    # 创建环境
    env: ManagerBasedRLEnv = ManagerBasedRLEnv(task_name=args.task, cfg=None, render_mode="rgb_array")
    env_cfg = env.cfg
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.scene.env_spacing = 3.0

    env.reset()

    # 取 action 维度并构造全 0 action
    action_dim = env.action_space.shape[0]
    zero_action = torch.zeros((env.num_envs, action_dim), device=env.device)

    print(f"[StandStillTest] num_envs={env.num_envs}, action_dim={action_dim}, device={env.device}")

    # 运行若干步
    for i in range(args.steps):
        obs, rew, terminated, truncated, info = env.step(zero_action)

        # 如果任何 env 终止，打印原因并 reset（你也可以选择直接 break）
        if torch.any(terminated) or torch.any(truncated):
            # 这里通常 terminated=base_contact/time_out 等
            print(f"[Step {i}] terminated={terminated.any().item()} truncated={truncated.any().item()}")
            env.reset()

    env.close()
    sim_app.close()

if __name__ == "__main__":
    main()
