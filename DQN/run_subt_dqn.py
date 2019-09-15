"""
Deep Q network,

Using:
Tensorflow: 1.0
gym: 0.7.3
"""


import gym
from RL_brain import DeepQNetwork

env = gym.make('gym_subt:subt-v0')
env = env.unwrapped

RL = DeepQNetwork(n_actions=len(env.actions),
                  n_features=env.laser_len,
                  learning_rate=0.01, e_greedy=0.95,
                  replace_target_iter=500, memory_size=4000,
                  e_greedy_increment=0.001,
                  save_graph_iter=10000)

total_steps = 0


for i_episode in range(5000):

    observation = env.reset()
    ep_r = 0
    while True:

        action = RL.choose_action(observation)

        observation_, reward, done, info = env.step(action)

       
        RL.store_transition(observation, action, reward, observation_)

        ep_r += reward
        if total_steps > 500:
            RL.learn()

        if done:
            print('episode: ', i_episode,
                  'ep_r: ', round(ep_r, 2),
                  'epsilon: ', round(RL.epsilon, 2))
            break

        observation = observation_
        total_steps += 1

RL.plot_cost()
