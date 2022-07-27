#!/usr/bin/env python

import cmd
import sys
import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from move_omorobot.srv import SetGoal, SetGoalResponse
from omo_r1_bringup.srv import *

import actionlib
from move_omorobot.msg import GoTargetOdomAction, GoTargetOdomFeedback, GoTargetOdomActionResult

GOAL_DIST_tolerance = 0.025
GOAL_ANGLE_tolerance = 0.025

MAX_Vx = 0.5        #maximum m/s
MAX_Vw = 0.5        #maximum rad/s
PID_Vw_Kp = 0.6     #P gain for aligning
PID_Vw_Ki = 0.05     #I gain for aligning
PID_Vx_Kp = 0.4     #P gain for linear V
PID_Vx_Ki = 0.05     #P gain for linear V

class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0

class MoveCommanderNode:
    def __init__(self):
        print("Hello Move_Commander_Node")
        #print(sys.version)
        rospy.Subscriber("/odom", Odometry, self.new_odom, queue_size=10)
        self.pub_cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Service('set_goal', SetGoal, self.service_setgoal)
        self.odom_new = OdomPose()
        self.odom_old = OdomPose()
        self.odom_origin = OdomPose()
        self.odom_goal = OdomPose()
        self.new_goal_set = False
        self.initial_angle_reached = False
        self.final_angle_reached = False
        self.goal_reached = False
        self.server = actionlib.SimpleActionServer('/go_target', GoTargetOdomAction, self.actionGoTarget, False)
        self.server.start()
        self.action_start = False
        self.PID_turn_I = 0.0
        self.PID_liear_I = 0.0
        self.initial_goal_angle = 0.0
        self.theta_slerp = 0
        self.time_to_goal = 0.0
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def actionGoTarget(self, goal):
        self.set_new_goal(goal.x, goal.y, goal.theta)
        self.action_start = True
        #self.server.set_preempted()
        rospy.loginfo('Action Set Goal: X= %.3f, Y=%.3f, Theta=%.3f',self.odom_goal.x, self.odom_goal.y, self.odom_goal.theta)

    def service_setgoal(self, req):
        self.set_new_goal(req.x, req.y, req.theta)
        #self.reset_odom(0.0, 0.0, 0.0)
        rospy.loginfo('Service Set Goal: X= %.3f, Y=%.3f, Theta=%.3f',self.odom_goal.x, self.odom_goal.y, self.odom_goal.theta)
        return SetGoalResponse()

    def set_new_goal(self, x, y, theta):
        self.odom_goal.x = x
        self.odom_goal.y = y
        self.odom_goal.theta = theta
        self.time_to_goal = 0.0
        # Compute initial goal angle (Pure pursuit)
        d_x_g = self.odom_goal.x - self.odom_new.x
        d_y_g = self.odom_goal.y - self.odom_new.y
        self.initial_goal_angle = math.atan2(d_y_g, d_x_g)
        self.new_goal_set = True
        self.goal_reached = False
        self.initial_angle_reached = False
        self.final_angle_reached = False
        self.PID_turn_I = 0.0
        self.PID_linear_I = 0.0

    def reset_odom(self, x,y,theta):
        rospy.wait_for_service('reset_odom')
        print("Service call reset_odom")
        try:
            resetOdom = rospy.ServiceProxy('reset_odom', ResetOdom)
            response = resetOdom(0.0, 0.0, 0.0)
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def timer_callback(self, event):
        cmdVel = Twist()
        cmdVel.linear.x = 0.0
        cmdVel.angular.z = 0.0
        if self.new_goal_set and not self.goal_reached:
            self.time_to_goal+=0.1
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
                    print "Finished in {0:.1f} sec".format(self.time_to_goal)
                    if self.action_start:
                        result = GoTargetOdomActionResult()
                        result.result = "Done"
                        self.server.set_succeeded(result)
                        self.action_start = False
                else:
                    if not self.final_angle_reached:
                        self.final_angle_reached = True
                        self.PID_turn_I = 0.0       #Reset PID
                    vw = self.pid_turn(d_angle)
                    cmdVel.angular.z = vw
            else:
                ## goal not reached to target POSITION
                if not self.initial_angle_reached:
                    # first align to the initial target angle
                    theta_error = self.initial_goal_angle - self.odom_new.theta
                    if abs(theta_error) > GOAL_ANGLE_tolerance:
                        vw = self.pid_turn(theta_error)
                        cmdVel.angular.z = vw
                    else :
                        self.initial_angle_reached = True
                        self.PID_turn_I = 0.0
                else:
                    # angle between current position to goal position relative to X axis
                    goal_angle = math.atan2(d_y_g, d_x_g)
                    d_angle = goal_angle - self.odom_new.theta
                    # For action server operation
                    if self.action_start:
                        self.send_feedback(dist_goal, d_angle)
                        
                    print "Dist:{0:.3f} D_angle:{1:.3f}".format(dist_goal, d_angle)
                    vw = self.pid_turn(d_angle)
                    cmdVel.angular.z = vw
                    #if abs(d_angle) > 0.1:
                    #    cmdVel.linear.x = 0.0
                    #else:
                    vx = self.pid_linear(dist_goal)
                    cmdVel.linear.x = vx
        self.pub_cmdVel.publish(cmdVel)

    def pid_turn(self, error):
        out = error * PID_Vw_Kp + self.PID_turn_I * PID_Vw_Ki
        self.PID_turn_I + error
        if out > MAX_Vw:
            out = MAX_Vw
        elif out < -MAX_Vw:
            out = -MAX_Vw
        return out

    def pid_linear(self, error):
        out = error * PID_Vx_Kp + self.PID_linear_I * PID_Vx_Ki
        self.PID_linear_I + error
        if out > MAX_Vx:
            out = MAX_Vx
        elif out < -MAX_Vx:
            out = -MAX_Vx
        return out

    def send_feedback(self, dist, theta):
        self.server.is_active()
        feedBack = GoTargetOdomFeedback()
        feedBack.dist = dist
        feedBack.theta = theta
        self.server.publish_feedback(feedBack)

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
    node = MoveCommanderNode()
    node.main()
