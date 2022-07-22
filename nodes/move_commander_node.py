#!/usr/bin/env python

import sys
import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from move_r1mini.srv import SetGoal, SetGoalResponse

GOAL_DIST_tolerance = 0.05
GOAL_ANGLE_tolerance = 0.05

MAX_Vx = 0.2        #maximum m/s
MAX_Vw = 0.3        #maximum rad/s

class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0

class MoveR1miniNode:
    def __init__(self):
        print("Hello MoveR1miniNode")
        #print(sys.version)
        rospy.Subscriber("/odom", Odometry, self.new_odom, queue_size=10)
        self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Service('set_goal', SetGoal, self.service_setgoal)
        self.odom_new = OdomPose()
        self.odom_origin = OdomPose()
        self.odom_goal = OdomPose()
        self.new_goal_set = False
        self.goal_reached = False
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def service_setgoal(self, req):
        self.odom_goal.x = req.x
        self.odom_goal.y = req.y
        self.odom_goal.theta = req.theta
        self.new_goal_set = True
        self.goal_reached = False
        rospy.loginfo('Set Goal: X= %.3f, Y=%.3f, Theta=%.3f',self.odom_goal.x, self.odom_goal.y, self.odom_goal.theta)
        return SetGoalResponse()

    def timer_callback(self, event):
        cmdVel = Twist()
        cmdVel.linear.x = 0.0
        cmdVel.angular.z = 0.0
        if self.new_goal_set and not self.goal_reached:
            ## Compute delta x,y,angle to target
            d_x_g = self.odom_goal.x - self.odom_new.x
            d_y_g = self.odom_goal.y - self.odom_new.y
            dist_goal = math.sqrt(d_x_g*d_x_g + d_y_g*d_y_g)
            d_angle = self.odom_goal.theta - self.odom_new.theta

            if dist_goal < GOAL_DIST_tolerance:
                ## goal reached to target POSITION and finaly turn to target ANGLE
                if abs(d_angle) < GOAL_ANGLE_tolerance:
                    self.goal_reached = True
                    self.new_goal_set = False
                else:
                    if d_angle > 0.0:
                        cmdVel.angular.z = MAX_Vw
                    else:
                        cmdVel.angular.z = -MAX_Vw
            else:
                ## goal not reached to target POSITION
                
                # angle between current position to goal position relative to X axis
                goal_angle = math.atan2(d_y_g, d_x_g)
                d_angle = goal_angle - self.odom_new.theta
                print "Dist:{0:.3f} D_angle:{1:.3f}%".format(dist_goal, goal_angle)
                if d_angle > 0.01:
                    cmdVel.angular.z = MAX_Vw
                elif d_angle < -0.01:
                    cmdVel.angular.z = -MAX_Vw
                if abs(d_angle) > 0.1:
                    cmdVel.linear.x = 0.0
                else:
                    cmdVel.linear.x = MAX_Vx
        self.pub_cmdVel.publish(cmdVel)

    def new_odom(self, msg):
        self.odom_new.x = msg.pose.pose.position.x
        self.odom_new.y = msg.pose.pose.position.y
        quat = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quat)
        self.odom_new.theta = euler[2]
        #rospy.loginfo('X= %.3f, Y=%.3f, Theta=%.3f', self.odom_new.x, self.odom_new.y, self.odom_new.theta)
        
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('move_r1mini_node')
    node = MoveR1miniNode()
    node.main()
