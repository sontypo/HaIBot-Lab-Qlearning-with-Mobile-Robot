#! /usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np

import sys
sys.path.insert(0, '/home/saun/ReinforcementLearning_Robot/src/Q_learning_Diffbot/scripts')

from Qlearning import *
from Lidar import *
from Control import *

ANGLE_MAX = 270
ANGLE_MIN = -270
# HORIZON_WIDTH = 135
HORIZON_WIDTH = 90

MIN_TIME_BETWEEN_SCANS = 0
MAX_SIMULATION_TIME = float('inf')

if __name__ == '__main__':
    try:
        state_space = createStateSpace()

        rospy.init_node('scan_node', anonymous = False)
        rate = rospy.Rate(10)

        now = datetime.now()
        dt_string_start = now.strftime("%d/%m/%Y %H:%M:%S")
        print('SCAN NODE START ==> ', dt_string_start ,'\r\n')

        scan_time = 0
        count = 0

        t_0 = rospy.Time.now()
        t_start = rospy.Time.now()

        # init timer
        while not (t_start > t_0):
            t_start = rospy.Time.now()

        t = t_start

        # Init figure - real time
        plt.style.use('seaborn-ticks')
        fig = plt.figure(1)
        ax = fig.add_subplot(1,1,1)

        # main loop
        while not rospy.is_shutdown():
            msgScan = rospy.wait_for_message('/scan', LaserScan)

            scan_time = (rospy.Time.now() - t).to_sec()
            sim_time = (rospy.Time.now() - t_start).to_sec()
            count = count + 1

            if scan_time > MIN_TIME_BETWEEN_SCANS:
                print('\r\nScan cycle:', count , '\r\nScan time:', scan_time, 's')
                print('Simulation time:', sim_time, 's')
                t = rospy.Time.now()

                # distances in [m], angles in [degrees]
                ( lidar, angles ) = lidarScan(msgScan)
                ( state_ind, s1, s2, s3 , s4, s5, s6, s7, s8, s9, a, b, c, d, e) = scanDiscretization(state_space, lidar)

                crash = checkCrash(lidar)
                object_nearby = checkObjectNearby(lidar)

                print('state index:', state_ind)
                print('s1 s2 s3 s4 s5 s6 s7 s8 s9')
                print(s1, '', s2, '', s3, '', s4, '', s5, '', s6, '', s7, '', s8, '', s9)
                if crash:
                    print('CRASH !')
                if object_nearby:
                    print('OBJECT NEARBY !')

                lidar_horizon = np.concatenate((lidar[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],lidar[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
                #angles_horizon = np.concatenate((angles[(ANGLE_MIN + HORIZON_WIDTH):(ANGLE_MIN):-1],angles[(ANGLE_MAX):(ANGLE_MAX - HORIZON_WIDTH):-1]))
                angles_horizon = np.linspace(90+HORIZON_WIDTH, 90-HORIZON_WIDTH, 180)

                # horizon in x-y plane
                x_horizon = np.array([])
                y_horizon = np.array([])
                for i in range(len(lidar_horizon)):
                    x_horizon = np.append(x_horizon,lidar_horizon[i] * np.cos(radians(angles_horizon[i])))
                    y_horizon = np.append(y_horizon,lidar_horizon[i] * np.sin(radians(angles_horizon[i])))

                ax.clear()
                plt.xlabel('distance[m]')
                plt.ylabel('distance[m]')
                plt.xlim((-1.8,1.8))
                plt.ylim((-1.8,1.8))
                plt.title('Lidar horizon')
                plt.axis('equal')
                ax.plot(x_horizon, y_horizon, 'b.', markersize = 8, label = 'obstacles')
                ax.plot(0, 0, 'r*', markersize = 20, label = 'robot')
                plt.legend(loc = 'lower right', shadow = True)
                plt.draw()
                plt.pause(0.0001)

            if sim_time > MAX_SIMULATION_TIME:
                now = datetime.now()
                dt_string_stop = now.strftime("%d/%m/%Y %H:%M:%S")
                print('\r\nSCAN NODE START ==> ', dt_string_start ,'\r\n')
                print('SCAN NODE STOP ==> ', dt_string_stop ,'\r\n')
                rospy.signal_shutdown('End of simulation')

            rate.sleep()

    except rospy.ROSInterruptException:
        now = datetime.now()
        dt_string_stop = now.strftime("%d/%m/%Y %H:%M:%S")
        print('\r\nSCAN NODE START ==> ', dt_string_start ,'\r\n')
        print('SCAN NODE STOP ==> ', dt_string_stop ,'\r\n')
        rospy.signal_shutdown('End of simulation')

        pass
