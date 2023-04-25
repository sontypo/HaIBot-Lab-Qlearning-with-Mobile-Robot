#! /usr/bin/env python3

#Viet chuong trinh dieu khien chuyen dong cho robot

import rospy
from time import time
from time import sleep
from datetime import datetime
import matplotlib.pyplot as plt
from geometry_msgs import *
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from robot_localization.srv import SetPose
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys
DATA_PATH = '/home/saun/ReinforcementLearning_Robot/src/Q_learning_Diffbot/Data'
MODULES_PATH = '/home/saun/ReinforcementLearning_Robot/src/Q_learning_Diffbot/scripts'
sys.path.insert(0, MODULES_PATH)

SAVING_DIR = '/home/saun/ReinforcementLearning_Robot/src/Q_learning_Diffbot/PoseData'


from Qlearning import *
from Lidar import *
from Control import *
from Pose_saving import *

# Real robot
REAL_ROBOT = False

# Action parameter
MIN_TIME_BETWEEN_ACTIONS = 0.0

# Initial and goal positions
INIT_POSITIONS_X = [ -0.7, -2.3, -0.5, -1, -2]
INIT_POSITIONS_Y = [ -0.7, -1.0, 1, -2, 1]
INIT_POSITIONS_THETA = [ 45, 0, -120, -90, 150]
GOAL_POSITIONS_X = [ 2.0, 5.0, 0.5, 1, 2]
GOAL_POSITIONS_Y = [ 1.0, 0.0, -1.9, 2, -1,]
GOAL_POSITIONS_THETA = [ 25.0, -90.0, -40, 60, -30,]

PATH_IND = 1

# Initial & Goal position
if REAL_ROBOT:
    X_INIT = 0.0
    Y_INIT = 0.0
    THETA_INIT = 0.0
    X_GOAL = 7.7
    Y_GOAL = 0.0
    THETA_GOAL = 0.0
else:
    RANDOM_INIT_POS = False

    X_INIT = INIT_POSITIONS_X[PATH_IND]
    Y_INIT = INIT_POSITIONS_Y[PATH_IND]
    THETA_INIT = INIT_POSITIONS_THETA[PATH_IND]

    # X_GOAL = GOAL_POSITIONS_X[PATH_IND]
    # Y_GOAL = GOAL_POSITIONS_Y[PATH_IND]
    # THETA_GOAL = GOAL_POSITIONS_THETA[PATH_IND]

# Log file directory - Q table source
Q_TABLE_SOURCE = DATA_PATH + '/2_Softmax_reconstruct_action_state_space_850eps_1500steps_90d'

def CreateRefPosePublic(X,Y,Theta):
    Ref_P = Pose()
    Ref_P.position.x=X
    Ref_P.position.y=Y
    Ref_P.position.z=0.0
    a,b,c,d = quaternion_from_euler(0.0,0.0,Theta)
    Ref_P.orientation.x=a
    Ref_P.orientation.y=b
    Ref_P.orientation.z=c
    Ref_P.orientation.w=d
    return Ref_P
    

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)

        setPosPub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        ref_pose = rospy.Publisher('/ref_pose',Pose, queue_size = 10)

        actions = createActions()
        state_space = createStateSpace()
        Q_table = readQTable(Q_TABLE_SOURCE+'/Qtable.csv')
        print('Initial Q-table:')
        print(Q_table)

        # Init time
        t_0 = rospy.Time.now()
        t_start = rospy.Time.now()

        # init timer
        while not (t_start > t_0):
            t_start = rospy.Time.now()

        t_step = t_start
        count = 0

        # robot in initial position
        robot_in_pos = False

         # set goal for robot
        goal_msg = rospy.wait_for_message('move_base_simple/goal', PoseStamped)
        # goal_msg = rospy.wait_for_message('/clicked_point', PointStamped)
        
        X_GOAL = goal_msg.pose.position.x
        Y_GOAL = goal_msg.pose.position.y
        z_goal = 0.0
        ort_q = goal_msg.pose.orientation
        ort_lst = [ort_q.x, ort_q.y, ort_q.z, ort_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(ort_lst)
        THETA_GOAL = degrees(yaw)
        GOAL = [X_GOAL,Y_GOAL,yaw]
        Ref_P = CreateRefPosePublic(X_GOAL,Y_GOAL, THETA_GOAL)
        ref_pose.publish(Ref_P)

        # because of the video recording
        sleep(1)

        # main loop
        while not rospy.is_shutdown():
            msgScan = rospy.wait_for_message('/scan', LaserScan)
            odomMsg = rospy.wait_for_message('/odom', Odometry)
            df = SavePoses(odomMsg)

            # np.savetxt(SAVING_DIR+'/Pose.csv', df, delimiter = ' , ')
            f =  open(SAVING_DIR+'/Pose_data3.csv', 'a')
            # f.write("fst\n")
            np.savetxt(f, df, delimiter = ' , ')
            f.close()

            # Secure the minimum time interval between 2 actions
            step_time = (rospy.Time.now() - t_step).to_sec()

            if step_time > MIN_TIME_BETWEEN_ACTIONS:
                t_step = rospy.Time.now()

                # f1 = open(SAVING_DIR+'/Pose_data3.csv', 'a')
                # f1.write("oke1\n")
                # np.savetxt(f1, df, delimiter = ' , ')
                # f.close()
               

                if not robot_in_pos:
                    robotStop(velPub)
                    # init pos
                    if REAL_ROBOT:
                        ( x_init , y_init , theta_init ) = (0, 0, 0)
                        odomMsg = rospy.wait_for_message('/odom', Odometry)
                        ( x , y ) = getPosition(odomMsg)
                        theta = degrees(getRotation(odomMsg))
                        robot_in_pos = True
                        print('\r\nInitial position:')
                        print('x = %.2f [m]' % x)
                        print('y = %.2f [m]' % y)
                        print('theta = %.2f [degrees]' % theta)
                        print('')
                    else:
                        if RANDOM_INIT_POS:
                            ( x_init , y_init , theta_init ) = robotSetRandomPos(setPosPub)
                        else:
                            ( x_init , y_init , theta_init ) = robotSetPos(setPosPub, X_INIT, Y_INIT, THETA_INIT)
                        # check init pos
                        # ( x_init , y_init , theta_init ) = (X_INIT, Y_INIT, THETA_INIT)
                        odomMsg = rospy.wait_for_message('/odom', Odometry)
                        ( x , y ) = getPosition(odomMsg)
                        theta = degrees(getRotation(odomMsg))
                        print(theta, theta_init)
                        if abs(x-x_init) < 0.05 and abs(y-y_init) < 0.05 and abs(theta-theta_init) < 2:
                            robot_in_pos = True
                            print('\r\nInitial position:')
                            print('x = %.2f [m]' % x)
                            print('y = %.2f [m]' % y)
                            print('theta = %.2f [degrees]' % theta)
                            print('')
                            sleep(1)
                        else:
                            robot_in_pos = False
                else:
                    count = count + 1
                    text = '\r\nStep %d , Step time %.2f s' % (count, step_time)

                    # with open(SAVING_DIR+'/Pose.csv', 'w+') as f:
                    # f2 =  open(SAVING_DIR+'/Pose_data3.csv', 'a')
                    # f2.write("oke2\n")
                    # np.savetxt(f2, df, delimiter = ' , ')
                    # f.close()

                    # Get robot position and orientation
                    ( x , y ) = getPosition(odomMsg)
                    theta = getRotation(odomMsg)

                    # Get lidar scan
                    # ( lidar, angles ) = lidarScan(msgScan)
                    # ( state_ind, x1, x2 ,x3 ,x4 ) = scanDiscretization(state_space, lidar)

                    # # Check for objects nearby
                    # crash = checkCrash(lidar)
                    # object_nearby = checkObjectNearby(lidar)
                    # goal_near = checkGoalNear(x, y, X_GOAL, Y_GOAL)
                    # enable_feedback_control = True

                    # # Stop the simulation
                    # if crash:
                    #     # robotStop(velPub)
                    #     # rospy.signal_shutdown('End of testing!')
                    #     # text = text + ' ==> Crash! End of simulation!'
                    #     # status = 'Crash! End of simulation!'

                    #     #<----------new---------->
                    #     robotStop(velPub)
                    #     #rospy.signal_shutdown('End of testing!')
                    #     text = text + ' ==> Crash! Go Around!!!'
                    #     status = 'Crash! Go Around!!!'
                    #     # rvelMsg = Twist()
                    #     # rvelMsg.linear.x = CONST_LINEAR_SPEED_AROUND
                    #     # rvelMsg.linear.y = 0
                    #     # rvelMsg.linear.z = 0
                    #     # rvelMsg.angular.x = 0
                    #     # rvelMsg.angular.y = 0
                    #     # rvelMsg.angular.z = CONST_ANGULAR_SPEED_AROUND

                    #     # Move around after crash
                    #     # rospy.sleep(5)
                    #     # CheckObjectLStatus(lidar)
                    #     robotGoAround(velPub)
                    #     # rospy.sleep(2)
                    #     rospy.Rate(50).sleep()
                    # Feedback control algorithm
                    # Get lidar scan
                    ( lidar, angles ) = lidarScan(msgScan)
                    ( state_ind, s1, s2, s3 , s4, s5, s6, s7, s8, s9, O_left, O_right, O_front, OF_left, OF_right) = scanDiscretization(state_space, lidar)

                    # Check for objects nearby
                    crash = checkCrash(lidar)
                    object_nearby = checkObjectNearby(lidar)
                    goal_near = checkGoalNear(x, y, X_GOAL, Y_GOAL)
                    enable_feedback_control = True

                    # Stop the simulation
                    cnt_l=0; cnt=0
                    if crash:
                        robotStop(velPub)
                       #rospy.signal_shutdown('End of testing!')
                        text = text + ' ==> Crash! Go Around!!!'
                        status = 'Crash! Go Around!!!'
                        # rvelMsg = Twist()
                        # rvelMsg.linear.x = CONST_LINEAR_SPEED_AROUND
                        # rvelMsg.linear.y = 0
                        # rvelMsg.linear.z = 0
                        # rvelMsg.angular.x = 0
                        # rvelMsg.angular.y = 0
                        # rvelMsg.angular.z = CONST_ANGULAR_SPEED_AROUND

                        # Move around after crash
                        # rospy.sleep(5)
                        # # CheckObjectLStatus(lidar)
                        
                        # if  (((O_left and O_front) and ( not OF_left )) or (O_left and O_front) or O_left) :
                        # # if  (((O_left and O_front) and ( not OF_left ))  or O_left or ()) :
                        #     # cnt=0
                        #     # while ((O_left and O_front) or O_left):
                        #     cnt_l=cnt_l + 1
                        #     print("left=", cnt_l)
                        #     for i in range(0, cnt_l+3, 1):
                        #         robotTurnRightRound(velPub)
                        #     # ( state_ind, x1, x2 ,x3 ,x4 ,O_left, O_right, O_front, OF_left, OF_right) = scanDiscretization(state_space, lidar)
                        #     # if  O_right:
                        #     #     cnt_l=0
                        #     #     break
                        #     # rospy.Rate(100).sleep()

                        # if (((O_right and O_front) and ( not OF_right )) or (O_right and O_front) or O_right):
                        # # if (((O_right and O_front) and ( not OF_right )) or O_right):  
                        #     # cnt=0
                        #     # while ((O_left and O_front) or O_left):
                        #     cnt=cnt + 1
                        #     print("right=", cnt)
                        #     for i in range(0, cnt+3, 1):
                        #         robotTurnLeftRound(velPub)
                        #     # ( state_ind, x1, x2 ,x3 ,x4 ,O_left, O_right, O_front, OF_left, OF_right) = scanDiscretization(state_space, lidar)
                        #     # rospy.Rate(100).sleep()
                        # robotGoAround(velPub)
                        # rospy.sleep(2)
                        rospy.Rate(100).sleep()


                    elif enable_feedback_control and ( not object_nearby or goal_near ):
                        status = robotFeedbackControl(velPub, x, y, theta, X_GOAL, Y_GOAL, radians(THETA_GOAL))
                        text = text + ' ==> Feedback control algorithm '
                        if goal_near:
                            text = text + '(goal near)'
                    # Q-learning algorithm
                    else:
                        ( action, status ) = getBestAction(Q_table, state_ind, actions)
                        if not status == 'getBestAction => OK':
                            print('\r\n', status, '\r\n')

                        status = robotDoAction(velPub, action)
                        if not status == 'robotDoAction => OK':
                            print('\r\n', status, '\r\n')
                        text = text + ' ==> Q-learning algorithm'

                    text = text + '\r\nx :       %.2f -> %.2f [m]' % (x, X_GOAL)
                    text = text + '\r\ny :       %.2f -> %.2f [m]' % (y, Y_GOAL)
                    text = text + '\r\ntheta :   %.2f -> %.2f [degrees]' % (degrees(theta), THETA_GOAL)

                    if status == 'Goal position reached!':
                        robotStop(velPub)
                        rospy.signal_shutdown('End of testing!')
                        text = text + '\r\n\r\nGoal position reached! End of simulation!'
                        print ("\n\n----------------------------------------------------------")
                        print (f"\nGOAL REACHED = {GOAL}")

                    print(text)
                
                

    except rospy.ROSInterruptException:
        robotStop(velPub)
        print('Simulation terminated!')
        pass
