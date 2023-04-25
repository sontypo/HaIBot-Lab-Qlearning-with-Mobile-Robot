#! /usr/bin/env python3

import numpy as np
from math import *
from std_msgs.msg import String
from itertools import product
from sensor_msgs.msg import LaserScan
import csv
import pandas as pd

STATE_SPACE_IND_MAX = 1200 
STATE_SPACE_IND_MIN = 0
ACTIONS_IND_MAX = 4 #2
ACTIONS_IND_MIN = 0

ANGLE_MAX = 270
ANGLE_MIN = -270
HORIZON_WIDTH = 90  #75 #135
# HORIZON_WIDTH = 90


T_MIN = 0.0001

# Create actions
def createActions():
    actions = np.array([0,1,2,3,4]) # [0,1,2]
    return actions

# Create fully state space for Q table
# def createStateSpace():
#     s1 = set((0,1,2,3))
#     s2 = set((0,1,2,3))
#     s3 = set((0,1,2))
#     s4 = set((0,1))
#     s5 = set((0,1))
#     s6 = set((0,1))
#     s7 = set((0,1))
#     s8 = set((0,1))
#     s9 = set((0,1))
#     state_space = set(product(s1,s2,s3,s4,s5,s6,s7,s8,s9))
#     return np.array(list(state_space))

# Handling State_space from available data
# def createStateSpace():
#     STATE_SPACE_DIR = "/home/saun/ReinforcementLearning_Robot/src/Q_learning_Diffbot"
#     file = open(STATE_SPACE_DIR+'/createState.csv', "r")
#     data = list(csv.reader(file, delimiter=","))
#     file.close()
#     state_from_csv = np.array(data)
#     state_space = []
#     for inner_state in state_from_csv:
#         inner_out_state = []
#         for string in inner_state:
#             inner_out_state.append(round(float(string)))
#         state_space.append(inner_out_state)
#     return np.array(state_space)

# Automatically generate State_space 
def createStateSpace():
    STATE_SPACE_DIR = "/home/saun/ReinforcementLearning_Robot/src/Q_learning_Diffbot"
    s1 = [0,1,2,3]
    s2 = [0,1,2,3]
    s3 = [0,1,2]
    s4 = [0,1]
    s5 = [0,1]
    s6 = [0,1]
    s7 = [0,1]
    s8 = [0,1]
    s9 = [0,1]
    available_state = list(product(s1, s2, s3, s4, s5, s6, s7, s8, s9))
    b = np.array(available_state)
    State = np.zeros(shape=b.shape, dtype="int")
    for i in range(0,len(b)):
     # Only Zone 1
        if (b[i][0]!=3 and b[i][1]==3 and b[i][2]==1 and b[i][6]==0 and b[i][7]==0 and b[i][8] == 0):
            State[i] = b[i]
     #Only Zone 2
        elif (b[i][0]==3 and b[i][1]!=3 and b[i][2]==2 and b[i][3]==0 and b[i][4]==0 and b[i][5]==0):
            State[i] = b[i]
      # None Obstacles
        elif (b[i][0] == 3 and b[i][1] == 3 and b[i][2] == 0 and b[i][3] == 0 and b[i][4] == 0 and b[i][5]==0 and b[i][6]==0 and b[i][7]==0 and b[i][8] == 0):
            State[i] = b[i]
     # Zone 1 + Zone 2
        elif (b[i][0] != 3 and b[i][1] != 3 and b[i][2] != 0):
            State[i] = b[i]
    df = pd.DataFrame(State)
    df1= df.loc[(df != 0).any(axis=1)]
    #Save State to excel
    df1.to_csv(STATE_SPACE_DIR+"/State.csv", index=False, header=False)
    file = open(STATE_SPACE_DIR+'/State.csv', "r")
    data = list(csv.reader(file, delimiter=","))
    file.close()
    state_from_csv = np.array(data)
    state_space = []
    for inner_state in state_from_csv:
        inner_out_state = []
        for string in inner_state:
            inner_out_state.append(round(float(string)))
        state_space.append(inner_out_state)
    return np.array(state_space)

# Create Q table, dim: n_states s n_actions
def createQTable(n_states, n_actions):
    #Q_table = np.random.uniform(low = -0.05, high = 0, size = (n_states,n_actions) )
    Q_table = np.zeros((n_states, n_actions))
    return Q_table

# Read Q table from path
def readQTable(path):
    Q_table = np.genfromtxt(path, delimiter = ' , ')
    return Q_table

# Write Q table to path
def saveQTable(path, Q_table):
    np.savetxt(path, Q_table, delimiter = ' , ')

# Select the best action a in state
def getBestAction(Q_table, state_ind, actions):
    if STATE_SPACE_IND_MIN <= state_ind <= STATE_SPACE_IND_MAX:
        status = 'getBestAction => OK'
        a_ind = np.argmax(Q_table[state_ind,:])
        a = actions[a_ind]
    else:
        status = 'getBestAction => INVALID STATE INDEX'
        a = getRandomAction(actions)

    return ( a, status )

# Select random action from actions
def getRandomAction(actions):
    n_actions = len(actions)
    a_ind = np.random.randint(n_actions)
    return actions[a_ind]

# Epsilog Greedy Exploration action chose
def epsiloGreedyExploration(Q_table, state_ind, actions, epsilon):
    if np.random.uniform() > epsilon and STATE_SPACE_IND_MIN <= state_ind <= STATE_SPACE_IND_MAX:
        status = 'epsiloGreedyExploration => OK'
        ( a, status_gba ) = getBestAction(Q_table, state_ind, actions)
        if status_gba == 'getBestAction => INVALID STATE INDEX':
            status = 'epsiloGreedyExploration => INVALID STATE INDEX'
    else:
        status = 'epsiloGreedyExploration => OK'
        a = getRandomAction(actions)

    return ( a, status )

# SoftMax Selection
def softMaxSelection(Q_table, state_ind, actions, T):
    if STATE_SPACE_IND_MIN <= state_ind <= STATE_SPACE_IND_MAX:
        status = 'softMaxSelection => OK'
        n_actions = len(actions)
        P = np.zeros(n_actions)

        # Boltzman distribution
        P = np.exp(Q_table[state_ind,:] / T) / np.sum(np.exp(Q_table[state_ind,:] / T))

        if T < T_MIN or np.any(np.isnan(P)):
            ( a, status_gba ) = getBestAction(Q_table, state_ind, actions)
            if status_gba == 'getBestAction => INVALID STATE INDEX':
                status = 'softMaxSelection => INVALID STATE INDEX'
        else:
            rnd = np.random.uniform()
            # rnd = np.random.choice(Q_table[state_ind,:])
            status = 'softMaxSelection => OK'
            if P[0] > rnd:
                a = 0
            elif P[0] <= rnd and (P[0] + P[1]) > rnd:
                a = 1
            elif (P[0] + P[1]) <= rnd and (P[0] + P[1] + P[2]) > rnd:
                a = 2
            elif (P[0] + P[1] + P[2]) <= rnd and (P[0] + P[1] + P[2] + P[3]) > rnd:
                a = 3
            elif (P[0] + P[1] + P[2] + P[3]) <= rnd:
                a = 4
            else:
                status = 'softMaxSelection => Boltzman distribution error => getBestAction '
                status = status + '\r\nP = (%f , %f , %f) , rnd = %f' % (P[0],P[1],P[2],P[3],P[4],rnd)
                status = status + '\r\nQ(%d,:) = ( %f, %f, %f) ' % (state_ind,Q_table[state_ind,0],Q_table[state_ind,1],Q_table[state_ind,2],Q_table[state_ind,3],Q_table[state_ind,4])
                ( a, status_gba ) = getBestAction(Q_table, state_ind, actions)
                if status_gba == 'getBestAction => INVALID STATE INDEX':
                    status = 'softMaxSelection => INVALID STATE INDEX'
    else:
        status = 'softMaxSelection => INVALID STATE INDEX'
        a = getRandomAction(actions)

    return ( a, status )

# # Reward function for Q-learning - table
# def getReward(action, prev_action, lidar, prev_lidar, crash):
#     if crash:
#         terminal_state = True
#         reward = -1000 # -100
#     else:
#         lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
#         prev_lidar_horizon = np.concatenate((prev_lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],prev_lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
#         terminal_state = False
#         # Reward from action taken = fowrad -> +0.2 , turn -> -0.1
#         if action == 0:
#             r_action = +5.0 # +0.2
#         elif action == 1 or action == 2:
#             r_action = +3.5 # -0.1
#         else:
#             r_action = +2.0 
#         # Reward from crash distance to obstacle change ( 1.7 to 1.9 tolerance in the metrics of the safety zone ) (older: 0.9 to 1.1)
#         W = np.linspace(1.7, 1.9, len(lidar_horizon) // 2)
#         W = np.append(W, np.linspace(1.9, 1.7, len(lidar_horizon) // 2))
#         if np.sum( W * ( lidar_horizon - prev_lidar_horizon) ) >= 0:
#             r_obstacle = +5.0 # +-0.2
#         else:
#             r_obstacle = -5.0
#         # Reward from turn left/right change
#         if ( prev_action == 1 and action == 2 ) or ( prev_action == 2 and action == 1 ):
#             r_change = -8.0 # -0.8 #8.0
#         # Reward from turn big_left/big_right change
#         elif ( prev_action == 3 and action == 4 ) or ( prev_action == 4 and action == 3 ):
#             r_change = -12.0 # 10.0
#         elif ( prev_action == 3 and action == 2 ) or ( prev_action == 2 and action == 3 ) or ( prev_action == 4 and action == 1 ) or ( prev_action == 1 and action ==4):
#             r_change = -10.0
#         else:
#             r_change = 0.0

#         # Cumulative reward
#         reward = r_action + r_obstacle + r_change

#     return ( reward, terminal_state )

# Reward function for Q-learning - table
def getReward(action, prev_action, lidar, prev_lidar, crash):
    if crash:
        terminal_state = True
        reward = -100 # -100
    else:
        lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
        prev_lidar_horizon = np.concatenate((prev_lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],prev_lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
        terminal_state = False
        # Reward from action taken = fowrad -> +0.2 , turn -> -0.1
        if action == 0:
            r_action = +0.4 # +0.2
        elif action == 1 or action == 2:
            r_action = +0.3 # -0.1
        else:
            r_action = +0.2 
        # Reward from crash distance to obstacle change ( 1.7 to 1.9 tolerance in the metrics of the safety zone ) (older: 0.9 to 1.1)
        W = np.linspace(1.7, 1.9, len(lidar_horizon) // 2)
        W = np.append(W, np.linspace(1.9, 1.7, len(lidar_horizon) // 2))
        if np.sum( W * ( lidar_horizon - prev_lidar_horizon) ) >= 0:
            r_obstacle = +0.5 # +-0.2
        else:
            r_obstacle = -0.5
        # Reward from turn left/right change
        if ( prev_action == 1 and action == 2 ) or ( prev_action == 2 and action == 1 ):
            r_change = -0.7 # -0.8 #8.0
        # Reward from turn big_left/big_right change
        elif ( prev_action == 3 and action == 4 ) or ( prev_action == 4 and action == 3 ):
            r_change = -0.9 # 10.0
        elif ( prev_action == 3 and action == 2 ) or ( prev_action == 2 and action == 3 ) or ( prev_action == 4 and action == 1 ) or ( prev_action == 1 and action ==4):
            r_change = -0.8
        else:
            r_change = 0.0

        # Cumulative reward
        reward = r_action + r_obstacle + r_change

    return ( reward, terminal_state )

# Update Q-table values
def updateQTable(Q_table, state_ind, action, reward, next_state_ind, alpha, gamma):
    if STATE_SPACE_IND_MIN <= state_ind <= STATE_SPACE_IND_MAX and STATE_SPACE_IND_MIN <= next_state_ind <= STATE_SPACE_IND_MAX:
        status = 'updateQTable => OK'
        Q_table[state_ind,action] = ( 1 - alpha ) * Q_table[state_ind,action] + alpha * ( reward + gamma * max(Q_table[next_state_ind,:]) )
    else:
        status = 'updateQTable => INVALID STATE INDEX'
    return ( Q_table, status )
