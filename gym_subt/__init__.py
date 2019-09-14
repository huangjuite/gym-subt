from gym.envs.registration import register

register(
    id='subt-v0',
    entry_point='gym_subt.envs:SubtEnv',
)
