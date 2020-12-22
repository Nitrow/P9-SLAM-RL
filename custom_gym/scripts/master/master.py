#!/usr/bin/env python3
import gym
import P9_RL_env_v01
import numpy as np
import torch as th
from P9_RL_env_v01.envs import P9RLEnv

from stable_baselines3 import PPO, SAC, DQN
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.sac import CnnPolicy
from stable_baselines3.ppo import CnnPolicy
from stable_baselines3.dqn import CnnPolicy

from stable_baselines3.common.cmd_util import make_vec_env
from stable_baselines3.common.env_checker import check_env

policy_kwargs = dict(activation_fn=th.nn.ReLU, net_arch=[256, 256, 256])


env = P9RLEnv()

model = DQN(CnnPolicy, env, tensorboard_log='/home/asger/P9/src/custom_gym/scripts', learning_starts=5000, target_update_interval=1000).learn(total_timesteps=2000000)

#model = PPO(CnnPolicy, env, verbose=1, batch_size=8, gamma=0.99, learning_rate=0.0001,
         #   tensorboard_log='/home/asger/P9/src/custom_gym/scripts')
# model = PPO.load(simLogPath +'models/'+ simRunID, env,  verbose=0, tensorboard_log=tensorBoard/)
#model.learn(total_timesteps=2000000, log_interval=1)
print('Training finished...')
model.save('/home/asger/P9/src/custom_gym/model')
print('Model saved...')

#from stable_baselines3 import TD3
#from stable_baselines3.td3.policies import MlpPolicy
#from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

#env = gym.make('P9_RL-v0')

# The noise objects for TD3
#n_actions = env.action_space.shape[-1]
#action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

#model = TD3(MlpPolicy, env, action_noise=action_noise, verbose=1)
#model.learn(total_timesteps=1000000, log_interval=10)
#model.save('/home/asger/P9/src/custom_gym/scripts')