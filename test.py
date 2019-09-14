import gym
import numpy as np

env = gym.make('gym_subt:subt-v0')
env = env.unwrapped
env.reset()

done = False
print env.laser_len
while not done:
    laser,reward,_,done = env.step(3)
    print len(laser.tolist())
    # print reward
env.reset()
env.close()