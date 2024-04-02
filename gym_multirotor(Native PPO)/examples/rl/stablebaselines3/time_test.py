import gym
import gym_multirotor
from stable_baselines3 import PPO
import time

# ENV_NAME = "QuadrotorXHoverEnv-v0"
ENV_NAME = "TiltrotorPlus8DofHoverEnv-v0"
# ENV_NAME = "QuadrotorPlusHoverEnv-v0"

model = PPO.load(f"./policy/PPO_{ENV_NAME}")
env = gym.make(ENV_NAME)
obs = env.reset()

# Duration of the simulation in seconds
simulation_duration = 2000
# Assuming each step in the environment is 1 second of simulation time.
# If it's different, adjust step_count accordingly.
step_count = simulation_duration

# Initialize a variable to store the total runtime
total_runtime = 0

# Run the simulation for 20 seconds
for _ in range(step_count):
    start_time = time.time()

    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)

    end_time = time.time()
    step_runtime = end_time - start_time
    total_runtime += step_runtime

    env.render()
    if dones:
        obs = env.reset()


# After the loop
print(f"Total Runtime for 20-second simulation: {total_runtime:.6f} seconds")
