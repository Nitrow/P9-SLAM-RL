#!/usr/bin/env python3
import gym
import P9_RL_env_v01
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.env_util import make_vec_env


env = gym.make('P9_RL-v0')


model = PPO1(MlpPolicy, env, verbose=0, gamma=0.99)
#model = PPO1.load(simLogPath +'models/'+ simRunID, env,  verbose=0, tensorboard_log=tensorboardPath) 
model.learn(total_timesteps=timeStepsToTrain, log_interval=0)
print('Training finished...')
print('Model saved...')
