from bluerov2_pipe_following.env.pipe_env_api import PipeEnv
import stable_baselines3

import os
from pathlib import Path

import gym
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':
    env = PipeEnv()

    logdir = "./log/"
    env = Monitor(env, filename=logdir, allow_early_resets=True)
    env = DummyVecEnv([lambda: env])

    # model = PPO("CnnPolicy", env, n_steps=1024, verbose=1)
    model = PPO.load("./log-14-02-23/rl_model_11000_steps.zip", env)
    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./log/', name_prefix='rl_model')
    model.learn(total_timesteps=int(1e5), callback=checkpoint_callback)
