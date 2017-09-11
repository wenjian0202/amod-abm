"""
environment class for the deep Q network (DQN)
"""

from lib.Agents import *
from lib.Demand import *
from lib.Constants import *

import gym
from gym import spaces
import copy
import itertools

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from rl.agents.dqn import DQNAgent
from rl.policy import EpsGreedyQPolicy
from rl.memory import SequentialMemory

class RebalancingEnv(gym.Env):
    """
    RebalancingEnv is the environment class for DQN
    Attributes:
        model: AMoD system to train
        dT: time interval for training
        penalty: penalty of rebalancing a vehicle
        action_space: action space
        state: the system state
        center: the centroid of cells
        input_dim: input dimension
    """ 
    def __init__(self, model, penalty=-10):
        self.model = model
        self.frames = []
        self.dT = INT_REBL
        self.penalty = penalty
        self.action_space = spaces.Discrete(9)
        self.state = np.zeros((3, Mlng, Mlat))
        self.center = np.zeros((Mlng, Mlat, 2))
        self.input_dim = 3 * Mlng * Mlat
        self.step_count = 0
        self.epi_count = 0
        self.total_reward = 0.0
        
    def step(self, action):
        self.step_count += 1
        model_ = copy.deepcopy(self.model)
        self.act(action)
        reward = 0 if action == 0 else self.penalty
        flag = False
        T = self.model.T
        T_ = self.model.T+INT_REBL
        while T < T_:
            T += INT_ASSIGN
            self.model.dispatch_at_time(None, T)
            # self.frames.append(copy.deepcopy(self.model.vehs))
            model_.dispatch_at_time(None, T)
            if not self.is_vehicle_idle():
                reward += model_.get_total_cost() - self.model.get_total_cost()
                flag = True
                break
        while T < T_:
            T += INT_ASSIGN
            self.model.dispatch_at_time(None, T)
            # self.frames.append(copy.deepcopy(self.model.vehs))  
        while not self.is_vehicle_idle():
            T = self.model.T
            T_ = self.model.T+INT_REBL
            while T < T_:
                T += INT_ASSIGN
                self.model.dispatch_at_time(None, T)
                # self.frames.append(copy.deepcopy(self.model.vehs))
        if flag:
            self.epi_count += 1
            self.total_reward += reward
            print("step %d; T: %d; average reward: %.2f - action: %s; reward: %.2f" % (self.step_count, self.model.T, self.total_reward/self.epi_count,
                                                     "noop" if action == 0 else 
                                                     "ne" if action == 1 else
                                                     "e" if action == 2 else
                                                     "se" if action == 3 else
                                                     "s" if action == 4 else
                                                     "sw" if action == 5 else
                                                     "w" if action == 6 else
                                                     "nw" if action == 7 else
                                                     "n" if action == 8 else "error!", reward))
        self.update_state()
        # print(self.state)
        return self.state, reward, flag, {}
    
    def act(self, action, vid=-1):
        veh = self.model.vehs[vid]
        self.model.act(None, veh, action, self.center)
    
    def reset(self):
        self.update_state()
        # self.amods.append( copy.deepcopy(self.amod) )
        return self.state
    
    def is_vehicle_idle(self, vid=-1):
        return self.model.vehs[vid].idle
    
    def update_state(self, vid=-1):
        veh = self.model.vehs[vid]
        self.state, self.center = self.model.get_state(veh)