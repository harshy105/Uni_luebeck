#!/usr/bin/env/ python
import gym
import numpy as np

class Q_Learner(object):
    def __init__(self, env):
        self.obs_shape = env.observation_space.shape #(2,)
        self.obs_high = env.observation_space.high #[0.6, 0.07]
        self.obs_low = env.observation_space.low #[-1.2, -0.07]
        self.obs_bins_d1 = 20  # Number of bins to Discretize in dim 1
        self.obs_bins_d2 = 15    # Number of bins to Discretize in dim 2 -velocity, should be odd number!
        self.obs_bins = np.array([self.obs_bins_d1,self.obs_bins_d2])
        self.bin_width = (self.obs_high - self.obs_low) / self.obs_bins #[0.09, 0.00933333]
        self.action_shape = env.action_space.n  #3

        # Q-values, Initialize the Q table with 1e-7 , in the last task, you can initialize it with 0 and compare the results,
        # for task III and question III with alpha = 1/#visit
        self.Q = np.ones((self.obs_bins[0] + 1, self.obs_bins[1] + 1, self.action_shape)) * 0 #(-1e7) # (21x 16 x 3)
        # Initialize the visit_counts
        self.visit_counts = np.zeros((self.obs_bins[0] + 1, self.obs_bins[1] + 1, self.action_shape)) # (21x 16 x 3)
        self.alpha = 0.05  # Learning rate
        self.gamma = 1.0  # Discount factor
        self.epsilon = 1.0 # Initialzation of epsilon value in epsilon-greedy

    def discretize(self, obs):
        '''A function maps the continuous state to discrete bins
        '''
        return tuple(((obs - self.obs_low) / self.bin_width).astype(int))

    def get_action(self, state):
        # dicreteize the observation first
        discretized_state = self.discretize(state)
        ''' 
            Implement the behavior policy (episilon greedy policy) based on the discretized_state 
            return the discrete action index
        '''
        if np.random.random() < self.epsilon: 
            action = np.random.choice(self.action_shape)  
        else:
            values = np.array([self.Q[discretized_state[0], discretized_state[1], a] for a in range(self.action_shape)]) # error: discretized_state is tuple and Q doesn't take tuple as inpute
            action = np.argmax(values)
        return action


    def update_Q_table(self, obs, action, reward, done, next_obs):
        '''update the Q table self.Q given each state,action ,reward... 
           Directly update the self.Q here and other necessary variables here.
        '''
        if not done: 
            state = self.discretize(obs)
            state_ = self.discretize(next_obs)
            values_ = np.array([self.Q[state_[0], state_[1], a] for a in range(self.action_shape)]) # error: discretized_state is tuple and Q doesn't take tuple as inpute
            self.Q[state[0],state[1],action] = self.Q[state[0],state[1],action] + self.alpha*(reward + self.gamma*values_.max() - self.Q[state[0],state[1],action])

        

def train(agent, env, MAX_NUM_EPISODES):
    
    episodic_returns = np.zeros(MAX_NUM_EPISODES)
    best_reward = -float('inf')
    for episode in range(MAX_NUM_EPISODES):
        # initialize the state
        obs = env.reset()
        # update the epsilon for decaying epsilon-greedy exploration
        agent.epsilon = agent.epsilon - 1/MAX_NUM_EPISODES 
        # initialization of the following variables
        episodic_return = 0
        done = False
        while not done:
            # (1) Select an action for the current state
            # (2) Interact with the environment, get the necessary info calling 
            # (3) Update the Q tables
            # (4) record the episodic cumulative reward
            # (5) Update the visit_counts per state-action pair  
            action = agent.get_action(obs)
            next_obs, reward, done, info = env.step(action)
            agent.update_Q_table(obs, action, reward, done, next_obs)
            episodic_return += reward
            discretized_state = agent.discretize(obs)
            agent.visit_counts[discretized_state[0], discretized_state[1], action] += 1
            obs = next_obs            
        
        episodic_returns[episode] = episodic_return
        if episodic_return > best_reward:
            best_reward = episodic_return   
        print("Episode#:{} reward:{} best_reward:{} eps:{}".format(episode, 
                                     episodic_return, best_reward, agent.epsilon))
    # Return the trained policy
    policy = np.argmax(agent.Q, axis=2)
    return policy, agent.Q.copy(), agent.visit_counts.copy(), episodic_returns


def test(agent, env, policy):
    # initialize the state
    obs = env.reset()
    # initialization of the following variables
    episodic_return = 0
    done = False
    episodic_length = 1000
    itr = 0
    while ((not done) & (itr < episodic_length)):
        action = agent.get_action(obs)
        next_obs, reward, done, info = env.step(action)
        episodic_return += reward
        obs = next_obs  
        itr += 1

    return episodic_return


if __name__ == "__main__":

    env = gym.make('MountainCar-v0').env # Note: the episode only terminates when cars reaches the target, the max episode length is not clipped to 200 steps.
    MAX_NUM_EPISODES = 2000
    agent = Q_Learner(env)
    learned_policy, Q, visit_counts, episodic_returns = train(agent, env, MAX_NUM_EPISODES)
    
    # after training, test the policy 10 times.
    for _ in range(10):
        reward = test(agent, env, learned_policy)
        print("Test reward: {}".format(reward))
    
    np.save('Q_harsh.npy',episodic_returns)
