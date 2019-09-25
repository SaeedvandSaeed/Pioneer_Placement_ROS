#!/usr/bin/env python3

import rospy
import numpy as np
from time import sleep
from geometry_msgs.msg import Pose2D
from pioneer_simulation.msg import Pose2DArray


class Placement_Simulation:
    def __init__(self):
        rospy.init_node('pioneer_placement_simulation', anonymous=False)
        rospy.loginfo("[PS] Pioneer Main Placement Simulation- Running")

        self.total_points      = 100
        self.main_rate         = rospy.Rate(60)
        self.keyboard_position = {'x': None, 'y': None, 'theta': None}

        # Publisher
        self.left_arm_points_pub  = rospy.Publisher("/pioneer/placement/left_arm_points",  Pose2DArray, queue_size=1)
        self.right_arm_points_pub = rospy.Publisher("/pioneer/placement/right_arm_points", Pose2DArray, queue_size=1)

        # Subscriber
        rospy.Subscriber("/pioneer/placement/keyboard_position", Pose2D, self.keyboard_position_callback)

    def keyboard_position_callback(self, msg):
        self.keyboard_position['x']     = msg.x
        self.keyboard_position['y']     = msg.y
        self.keyboard_position['theta'] = msg.theta

        rospy.loginfo("[PS] Keyboard Position : {}".format(self.keyboard_position))

    def pack_points(self, group, num_point, x, y):
        points      = Pose2DArray()
        points.name = group

        for i in range (num_point):
            point   = Pose2D()
            point.x = x[i]
            point.y = y[i]
            points.poses.append(point)
        return points

    def run(self):
        # Samples random left arm points
        np.random.seed(0)
        xl_trajectory = np.random.randint(640, size = self.total_points)
        yl_trajectory = np.random.randint(480, size = self.total_points)
        # print('Left X Points: {}'.format(xl_trajectory))
        # print('Left Y Points: {}'.format(yl_trajectory))

        # Samples random right arm points
        np.random.seed(1)
        xr_trajectory = np.random.randint(640, size = self.total_points)
        yr_trajectory = np.random.randint(480, size = self.total_points)
        # print('Right X Points: {}'.format(xr_trajectory))
        # print('Right Y Points: {}'.format(yr_trajectory))

        left_arm_points  = self.pack_points(self, 'left_arm',  self.total_points, xl_trajectory, yl_trajectory)
        right_arm_points = self.pack_points(self, 'right_arm', self.total_points, xr_trajectory, yr_trajectory)
        
        while not rospy.is_shutdown():
            scan = input('Please input : ')
            if scan == "send":
                self.left_arm_points_pub.publish(left_arm_points)
                self.right_arm_points_pub.publish(right_arm_points)
            elif scan == "exit":
                break                
            else:
                rospy.loginfo("[PS] Wrong input")

            self.main_rate.sleep()

if __name__ == '__main__':
    ps = Placement_Simulation()
    ps.run()