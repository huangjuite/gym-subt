import gym

env = gym.make('gym_subt:subt-v0')
env = env.unwrapped
env.reset()

done = False

while not done:
    _,reward,_,done = env.step(3)
    print reward
env.reset()
env.close()