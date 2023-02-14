from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.callbacks import CheckpointCallback
import gym
import numpy as np
import matplotlib.pyplot as plt
import os
from matplotlib import rc
from bluerov2_pipe_following.env.pipe_env_api import PipeEnv


env = PipeEnv()

env = DummyVecEnv([lambda: env])

model = PPO.load("./log-14-02-23/rl_model_11000_steps.zip")

episode_times = np.zeros((10, 20))
finish_passed = np.zeros((10, 20))
final_position = np.zeros((10, 20))

for route_count in range(10):
    print("current route: ", route_count)
    episode_finish_count = 0
    for episode_count in range(10):
        print("episode: ", episode_count)
        obs = env.envs[0].reset()
        for i in range(5002):  # breaks at the end of the episode
            action, _states = model.predict(obs, deterministic=True)
            obs, rewards, dones, info = env.envs[0].step(action)
            if dones:
                break
    print("completed over 20 run: ", episode_finish_count)

print(episode_times)
print(finish_passed)
np.save("ep_times", episode_times)
np.save("finish_passed", finish_passed)
