import gym
import gym_multirotor
from stable_baselines3 import PPO
import sys
import os
import torch


# Adjust the relative path to reach the 'ppo' directory from your script's location
relative_path_to_ppo = '../experiments_from_paper/ppo'

# Get the absolute path to the 'ppo' directory
absolute_path_to_ppo = os.path.join(os.path.dirname(__file__), relative_path_to_ppo)
absolute_path_to_ppo = os.path.normpath(absolute_path_to_ppo)  # Normalize the path to resolve any ".."

# Add the 'ppo' directory to the Python path
sys.path.append(absolute_path_to_ppo)

from envs import make_vec_envs  # Assuming you want to import make_env from envs.py

def main():
    # These parameters are taken from the example command line call
    seed = 123
    env_name = "TiltrotorPlus8DofHoverEnv-v0"
    num_processes = 8
    gamma = 0.99
    log_dir = f'logs/log_{env_name}'
    save_dir = f"./policy/PPO_{env_name}"
    device = torch.device("cuda")  # Since --no-cuda is specified
    allow_early_resets = not True  # Since --use-proper-time-limits is specified (inverting logic)
   
    # Create the vectorized environment with the custom function
    vec_env = make_vec_envs(
        env_name=env_name,
        seed=seed,
        num_processes=num_processes,
        gamma=gamma,
        log_dir=log_dir,
        device=device,
        allow_early_resets=allow_early_resets
    )


    model = PPO(
        "MlpPolicy",
        vec_env,
        verbose=1,
        tensorboard_log=log_dir,
        policy_kwargs=dict(activation_fn=torch.nn.ReLU, net_arch=dict(pi=[256, 256], vf=[256, 256])),
        learning_rate=0.00005,
        clip_range=0.05,
        seed=seed,
        batch_size=256,
        max_grad_norm=0.2
    )
    model.learn(total_timesteps=40000000)
    model.save(save_dir)
    del model
    vec_env.close()

if __name__ == '__main__':
    main()


