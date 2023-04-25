#! /usr/bin/env python3

import numpy as np
from math import *
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from itertools import product


MAX_LIDAR_DISTANCE = 1.8
COLLISION_DISTANCE = 0.55 # LaserScan.range_min = 0.5
NEARBY_DISTANCE = 0.85

ZONE_0_LENGTH = 0.75
ZONE_1_LENGTH = 1.4

ANGLE_MAX = 270
ANGLE_MIN = -270
# HORIZON_WIDTH = 135   #75
HORIZON_WIDTH = 90


# Convert LasecScan msg to array
def lidarScan(msgScan):
    distances = np.array([])
    angles = np.array([])

    for i in range(len(msgScan.ranges)):
        angle = degrees(i * msgScan.angle_increment)
        if ( msgScan.ranges[i] > MAX_LIDAR_DISTANCE ):
            distance = MAX_LIDAR_DISTANCE
        elif ( msgScan.ranges[i] < msgScan.range_min ):
            distance = msgScan.range_min
            # For real robot - protection
            if msgScan.ranges[i] < 0.01:
                distance = MAX_LIDAR_DISTANCE
        else:
            distance = msgScan.ranges[i]

        distances = np.append(distances, distance)
        angles = np.append(angles, angle)

    # distances in [m], angles in [degrees]
    return ( distances, angles )

# Discretization of lidar scan
def scanDiscretization(state_space, lidar):
    s1 = 3 # Left zone (no obstacle detected) s1, s2, s.....
    s2 = 3 # Right zone (no obstacle detected)
    s3 = 0
    s4 = 0 # Left sector (no obstacle detected)
    s5 = 0
    s6 = 0
    s7 = 0
    s8 = 0
    s9 = 0 # Right sector (no obstacle detected)
        
    # Find the left side lidar values of the vehicle
    lidar_left = min(lidar[(ANGLE_MIN):(ANGLE_MIN + HORIZON_WIDTH)], default=0)
    if MAX_LIDAR_DISTANCE > lidar_left> ZONE_1_LENGTH:
        s1 = 2
    elif ZONE_1_LENGTH > lidar_left > ZONE_0_LENGTH:
        s1 = 1 # zone 1
    elif lidar_left <= ZONE_0_LENGTH:
        s1 = 0 # zone 0

    # Find the right side lidar values of the vehicle
    lidar_right = min(lidar[(ANGLE_MAX - HORIZON_WIDTH):(ANGLE_MAX)], default=0)
    if MAX_LIDAR_DISTANCE > lidar_right > ZONE_1_LENGTH:
        s2 = 2
    elif ZONE_1_LENGTH > lidar_right > ZONE_0_LENGTH:
        s2 = 1 # zone 1
    elif lidar_right <= ZONE_0_LENGTH:
        s2 = 0 # zone 0
        
    # Detection of object in front of the robot
    if ( min(lidar[(ANGLE_MAX - HORIZON_WIDTH // 3):(ANGLE_MAX)], default=0) < 1.8 ) or ( min(lidar[(ANGLE_MIN):(ANGLE_MIN + HORIZON_WIDTH // 3)], default=0) < 1.8 ):
        object_front = True
    else:
        object_front = False

    # Detection of object on the left side of the robot
    if min(lidar[(ANGLE_MIN):(ANGLE_MIN + 2 * HORIZON_WIDTH // 3)], default=0) < 1.8:
        object_left = True
    else:
        object_left = False

    # Detection of object on the right side of the robot
    if min(lidar[(ANGLE_MAX - 2 * HORIZON_WIDTH // 3):(ANGLE_MAX)], default=0) < 1.8:
        object_right = True
    else:
        object_right = False

    # Detection of object on the far left side of the robot
    if min(lidar[(ANGLE_MIN + HORIZON_WIDTH // 3):(ANGLE_MIN + HORIZON_WIDTH)], default=0) < 1.8:
        object_far_left = True
    else:
        object_far_left = False

    # Detection of object on the far right side of the robot
    if min(lidar[(ANGLE_MAX - HORIZON_WIDTH):(ANGLE_MAX - HORIZON_WIDTH // 3)], default=0) < 1.8:
        object_far_right = True
    else:
        object_far_right = False

    # The left sector of the vehicle
    if ( object_front and object_left ) and ( not object_far_left ):
        s6 = 1 # sector 0
    elif ( object_left and object_far_left ) and ( not object_front ):
        s5 = 1 # sector 1
    elif object_far_left:
        s4 = 1 # sector 2

    if ( object_front and object_right ) and ( not object_far_right ):
        s7 = 1 # sector 0
    elif ( object_right and object_far_right ) and ( not object_front ):
        s8 = 1 # sector 1
    elif object_far_right:
        s9 = 1 # sector 2
        
    # s3 difinitions
    if ( s1 <= 2 and s2 == 3 and s7 == 0 and s8 == 0 and s9 == 0 ) or ( s1 <=2 and s2 <= 2 and ( lidar_right > lidar_left ) ): # Left zone
        s3 = 1
    elif ( s1 == 3 and s2 <= 2 and s4 == 0 and s5 == 0 and s6 == 0 ) or ( s1 <=2 and s2 <= 2 and ( lidar_right < lidar_left ) ): # Right zone
        s3 = 2
        
    # Find the state space index of state's set in Q table
    ss = np.where(np.all(state_space == np.array([s1,s2,s3,s4,s5,s6,s7,s8,s9]), axis = 1))
    state_ind = int(ss[0])

    # return ( state_ind, s1, s2, s3 , s4 )
    return ( state_ind, s1, s2, s3 , s4, s5, s6, s7, s8, s9, object_left, object_right, object_front, object_far_left, object_far_right )


# Check - crash
def checkCrash(lidar):
    lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
    W = np.linspace(1.2, 1, len(lidar_horizon) // 2)
    W = np.append(W, np.linspace(1, 1.2, len(lidar_horizon) // 2))
    if np.min( W * lidar_horizon ) < COLLISION_DISTANCE:
        return True
    else:
        return False

# Check - object nearby
def checkObjectNearby(lidar):
    lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
    W = np.linspace(1.4, 1, len(lidar_horizon) // 2)
    W = np.append(W, np.linspace(1, 1.4, len(lidar_horizon) // 2))
    if np.min( W * lidar_horizon ) < NEARBY_DISTANCE:
        return True
    else:
        return False

# Check - goal near
def checkGoalNear(x, y, x_goal, y_goal):
    ro = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    if ro < 0.3:
        return True
    else:
        return False
