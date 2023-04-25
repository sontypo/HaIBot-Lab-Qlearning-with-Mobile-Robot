#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import time
import pandas as pd
import numpy as np

SAVING_DIR = '/home/saun/ReinforcementLearning_Robot/src/Q_learning_Diffbot/PoseData'

# odomMsg = rospy.wait_for_message('/odom', Odometry)
def SavePoses(odomMsg):
    pose_data = Pose()
    pose_data.position.x = odomMsg.pose.pose.position.x
    pose_data.position.y = odomMsg.pose.pose.position.y
    pose_data.position.z = 0.0
    pose_data.orientation.x = 0.0
    pose_data.orientation.y = 0.0
    pose_data.orientation.z = 0.0
    pose_data.orientation.w = 0.0

    a = pose_data.position.x
    b = pose_data.position.y

    df = pd.DataFrame([[a,b]])
    return df

    



# class SavePoses(object):
#     def __init__(self):
        
#         self._pose = Pose()
#         self.poses_dict = {"pose1":self._pose, "pose2":self._pose, "pose3":self._pose}
#         self._pose_sub = rospy.Subscriber('/odom', Odometry , self.sub_callback)
#         self.write_to_file()

#     def sub_callback(self, msg):
        
#         self._pose = msg.pose.pose
    
#     def write_to_file(self):
        
#         time.sleep(5)
#         self.poses_dict["pose1"] = self._pose
#         rospy.loginfo("Written pose1")
#         time.sleep(5)
#         self.poses_dict["pose2"] = self._pose
#         rospy.loginfo("Written pose2")
#         time.sleep(5)
#         self.poses_dict["pose3"] = self._pose
#         rospy.loginfo("Written pose3")
            
        
#         with open('poses.txt', 'w') as file:
            
#             for key, value in self.poses_dict.iteritems():
#                 if value:
#                     file.write(str(key) + ':\n----------\n' + str(value) + '\n===========\n')
                    
#         rospy.loginfo("Written all Poses to poses.txt file")