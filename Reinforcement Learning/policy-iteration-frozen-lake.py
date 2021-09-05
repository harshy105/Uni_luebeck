import numpy as np
import matplotlib.pyplot as plt
import gym

def mdp_to_mrp(pi, model):
    P_pi = np.zeros([env.nb_states, env.nb_states])  # transition probability matrix (s) to (s')
    R_pi = np.zeros([env.nb_states])             # exp. reward from state (s) to any next state
    for s in range(env.nb_states):
        a = pi[s]
        for p_, s_, r_, _ in model[s][a]:
            # p_ - transition probability from (s,a) to (s')
            # s_ - next state (s')
            # r_ - reward on transition from (s,a) to (s')
            P_pi[s, s_] +=  p_   # transition probability (s) -> (s')
            Rsa = p_ * r_        # exp. reward from (s,a) to any next state
            R_pi[s] +=  Rsa      # exp. reward from (s) to any next state
    assert np.alltrue(np.sum(P_pi, axis=-1)==np.ones([env.nb_states]))  # rows should sum to 1
    return P_pi, R_pi

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


def policy_iter(env, gamma, theta):
    """To Do : Implement Policy Iteration Algorithm
    gamma (float) - discount factor
    theta (float) - termination condition
    env - environment with following required memebers:
    
    Useful variables/functions:
        
            env.nb_states - number of states
            env.nb_actions - number of actions
            env.model     - prob-transitions and rewards for all states and actions, you can play around that
        
        
        return the value function V and policy pi, 
        pi should be a determinstic policy and an illustration of randomly initialized policy is below
    """
    # initialize the random policy
    pi = np.random.randint(low=0, high=env.action_space.n, size=env.nb_states)
    # Initialize the value function
    V = np.zeros(env.nb_states)
    V_old = np.ones(env.nb_states)
    # To do......
    
    while np.linalg.norm(V-V_old)>theta:
        V_old = V
        # Policy Evaluation (V) 
        P_pi, R_pi = mdp_to_mrp(pi, env.model)
        V = R_pi + gamma * P_pi @ V_old
        
        # Policy Improvement (pi)
        Q_pi = calc_Q_pi(V, env.model, gamma)
        pi = np.argmax(Q_pi, axis=-1)     # choose greedy action
    
    return V, pi


if __name__ == '__main__':
    env = gym.make('FrozenLake-v0')
    env.reset()
    env.render()
    
    if not hasattr(env, 'nb_states'):  
        env.nb_states = env.env.nS
    if not hasattr(env, 'nb_actions'): 
        env.nb_actions = env.env.nA
    if not hasattr(env, 'model'):      
        env.model = env.env.P

    # Check #state, #actions and transition model
    # env.model[state][action]
    print(env.nb_states, env.nb_actions, env.model[14][2])


    # display the result
        
    V, pi = policy_iter(env, gamma=1.0, theta=1e-5)
    print(V.reshape([4, -1]))    
    
    a2w = {0:'<', 1:'v', 2:'>', 3:'^'}
    policy_arrows = np.array([a2w[x] for x in pi])
    print(np.array(policy_arrows).reshape([-1, 4]))
    
#    print(env.model[14][2])
