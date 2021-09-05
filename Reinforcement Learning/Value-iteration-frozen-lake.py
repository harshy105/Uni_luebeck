import numpy as np
import gym
import matplotlib.pyplot as plt
env = gym.make('FrozenLake-v0')


def calc_Q_pi(V_pi, model, gamma):
    Q_pi=np.zeros([env.nb_states, env.nb_actions])
    for s in range(env.nb_states):
        for a in range(env.nb_actions):
            for p_, s_, r_, _ in model[s][a]:
                # p_ - transition probability from (s,a) to (s')
                # s_ - next state (s')
                # r_ - reward on transition from (s,a) to (s')
                Rsa = p_ * r_   # expected reward for transition s,a -> s_
                Vs_ = V_pi[s_]    # state-value of s_
                Q_pi[s, a] += Rsa + gamma * p_ * Vs_
    return Q_pi


def value_iter(env, gamma, theta):
    """To Do : Implement Policy Iteration Algorithm
    gamma (float) - discount factor
    theta (float) - termination condition
    env - environment with following required memebers:
    
    Useful variables/functions:
        
            env.nb_states - number of states
            env.nb_action - number of actions
            env.model     - prob-transitions and rewards for all states and actions, you can play around that
        
        
        return the value function V and policy pi, 
        pi should be a determinstic policy and an illustration of randomly initialized policy is below
    """
    # Initialize the value function
    V = np.zeros(env.nb_states)
    pi = np.random.randint(low=0, high=env.action_space.n, size=env.nb_states)
    # to do
    V_old = np.ones(env.nb_states)
    
    while np.linalg.norm(V-V_old)>theta:
        V_old = V
        Q_pi = calc_Q_pi(V_old, env.model, gamma) #calculate Q-values
        pi = np.argmax(Q_pi, axis=-1) # choose greedy action
        V = Q_pi[range(env.nb_states), pi]

    return V, pi, Q_pi

if __name__ == '__main__':
    env.reset()
    env.render()

    # Check #state, #actions and transition model
    # env.model[state][action]
    
    if not hasattr(env, 'nb_states'):  
        env.nb_states = env.env.nS
    if not hasattr(env, 'nb_actions'): 
        env.nb_actions = env.env.nA
    if not hasattr(env, 'model'):      
        env.model = env.env.P
        
    print(env.nb_states, env.nb_actions, env.model[14][2])
        
    V, pi,Q_pi = value_iter(env, gamma=1, theta=1e-4)
    print(V.reshape([4, -1]))
    
    
    a2w = {0:'<', 1:'v', 2:'>', 3:'^'}
    policy_arrows = np.array([a2w[x] for x in pi])
    print(np.array(policy_arrows).reshape([-1, 4]))
