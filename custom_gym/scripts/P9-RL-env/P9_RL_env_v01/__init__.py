from gym.envs.registration import register

register(
    id='P9_RL-v0',
    entry_point='P9_RL_env_v01.envs:P9RLEnv',
)
