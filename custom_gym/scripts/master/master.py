#!/usr/bin/env python3
import gym
import P9_RL_env_v01
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.env_util import make_vec_env

env = make_vec_env('P9_RL-v0', n_envs=2)


model = PPO(MlpPolicy, env, verbose=0, gamma=0.99)
#model = PPO1.load(simLogPath +'models/'+ simRunID, env,  verbose=0, tensorboard_log=tensorboardPath) 
model.learn(total_timesteps=10000)
print('Training finished...')
print('Model saved...')
