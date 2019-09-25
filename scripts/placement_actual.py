#!/usr/bin/env python3

import rospy
import numpy as np
from time import sleep
from geometry_msgs.msg import Pose2D
from pioneer_simulation.msg import Pose2DArray


class Placement_Actual:
    def __init__(self):
        rospy.init_node('pioneer_placement_actual', anonymous=False)
        rospy.loginfo("[PA] Pioneer Main Placement Actual- Running")

        self.left_points = None
    
        # Publisher
        self.keyboard_pos_pub = rospy.Publisher("/pioneer/placement/keyboard_position", Pose2D, queue_size=1)

        # Subscriber
        rospy.Subscriber("/pioneer/placement/left_arm_points",  Pose2DArray, self.left_arm_points_callback)
        rospy.Subscriber("/pioneer/placement/right_arm_points", Pose2DArray, self.right_arm_points_callback)
    
    def left_arm_points_callback(self, msg):
        group            = msg.name
        num_points       = len(msg.poses)
        self.left_points = msg.poses
        rospy.loginfo('[PA] Group: {}, Total_Points: {}, Points: {}'.format(group, num_points, self.left_points))
        # rospy.loginfo('[PA] Group: {}, Total_Points: {}, Points: {}'.format(group, num_points, self.left_points[:10]))

    def right_arm_points_callback(self, msg):
        group             = msg.name
        num_points        = len(msg.poses)
        self.right_points = msg.poses
        rospy.loginfo('[PA] Group: {}, Total_Points: {}, Points: {}'.format(group, num_points, self.right_points))
        # rospy.loginfo('[PA] Group: {}, Total_Points: {}, Points: {}'.format(group, num_points, self.right_points[:10]))

    def run(self):
        while not rospy.is_shutdown():
            pass
            scan = input('Please input : ')
            if scan == "send":
                keyboard       = Pose2D()
                keyboard.x     = 520
                keyboard.y     = 200
                keyboard.theta = 2
                self.keyboard_pos_pub.publish(keyboard)
            
            elif scan == "exit":
                break                
            else:
                rospy.loginfo("[PA] Wrong input")

if __name__ == '__main__':
    pa = Placement_Actual()
    pa.run()