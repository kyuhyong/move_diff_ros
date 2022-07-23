import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from move_commander_interface.srv import SetGoal
from move_commander_interface.action import GoTargetOdom
from tf_transformations import euler_from_quaternion
import math

GOAL_DIST_tolerance = 0.05
GOAL_ANGLE_tolerance = 0.05

MAX_Vx = 0.1        #maximum m/s
MAX_Vw = 0.4        #maximum rad/s

class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0

class MoveCommanderNode(Node):
    def __init__(self):
        super().__init__('move_commander')
        self.callback_group = ReentrantCallbackGroup()
        #self.callback_group_timer = ReentrantCallbackGroup()
        ## Create msg subscription
        self.sub_odom = self.create_subscription(Odometry,'/odom', self.new_odom, 10 )
        ## Create msg publisher
        self.pub_cmdVel = self.create_publisher(Twist,'/cmd_vel', 10)
        ## Create service
        self.srv_setGoal = self.create_service(SetGoal, '/set_goal', self.new_setGoal)        
        ## Create action server
        self.act_goTarget = ActionServer(self, GoTargetOdom, '/go_to', self.act_goTarget,
            callback_group=self.callback_group)
        
        timer_period = 0.1  # seconds
        self.timer_count = 0
        self.odom_new = OdomPose()
        self.odom_origin = OdomPose()
        self.odom_goal = OdomPose()
        self.new_goal_set = False
        self.goal_reached = False
        self.odom_ready = False
        self.dist_to_goal = 0.0
        self.angle_to_goal = 0.0
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.callback_group)

    def new_setGoal(self, request, response):
        self.odom_goal.x = request.x
        self.odom_goal.y = request.y
        self.odom_goal.theta = request.theta
        self.new_goal_set = True
        self.goal_reached = False
        print(f"New Goal: X= {self.odom_goal.x:.3f}, Y={self.odom_goal.y:.3f}, Theta={self.odom_goal.theta:.3f}")
        return response
    
    def act_goTarget(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.odom_goal.x = goal_handle.request.x
        self.odom_goal.y = goal_handle.request.y
        self.odom_goal.theta = goal_handle.request.theta
        self.new_goal_set = True
        self.goal_reached = False
        print(f"New Goal: X= {self.odom_goal.x:.3f}, Y={self.odom_goal.y:.3f}, Theta={self.odom_goal.theta:.3f}")
        feedback_msg = GoTargetOdom.Feedback()
        while not self.goal_reached:
            feedback_msg.dist = self.dist_to_goal
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        goal_handle.succeed()
        result = GoTargetOdom.Result()
        result.reached = True
        return result

    def timer_callback(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        if self.odom_ready and self.new_goal_set and not self.goal_reached:
            ## Compute delta x,y,angle to target
            d_x_g = self.odom_goal.x - self.odom_new.x
            d_y_g = self.odom_goal.y - self.odom_new.y
            self.dist_to_goal = math.sqrt(d_x_g*d_x_g + d_y_g*d_y_g)
            d_angle = self.odom_goal.theta - self.odom_new.theta

            if self.dist_to_goal < GOAL_DIST_tolerance:
                ## goal reached to target POSITION and finaly turn to target ANGLE
                if abs(d_angle) < GOAL_ANGLE_tolerance:
                    self.goal_reached = True
                    self.new_goal_set = False
                else:
                    if d_angle > 0.0:
                        cmd_vel.angular.z = MAX_Vw
                    else:
                        cmd_vel.angular.z = -MAX_Vw
            else:
                ## goal not reached to target POSITION
                
                # angle between current position to goal position relative to X axis
                goal_angle = math.atan2(d_y_g, d_x_g)
                d_angle = goal_angle - self.odom_new.theta
                print(f"Dist:{self.dist_to_goal:.3f} D_angle:{goal_angle:.3f}")
                if d_angle > 0.01:
                    cmd_vel.angular.z = MAX_Vw
                elif d_angle < -0.01:
                    cmd_vel.angular.z = -MAX_Vw
                if abs(d_angle) > 0.1:
                    cmd_vel.linear.x = 0.0
                else:
                    cmd_vel.linear.x = MAX_Vx
        self.pub_cmdVel.publish(cmd_vel)
        self.timer_count+=1
        #self.get_logger().info('Publishing: "%s"' % msg.data)

    def new_odom(self, msg):
        self.odom_new.x = msg.pose.pose.position.x
        self.odom_new.y = msg.pose.pose.position.y
        quat = [msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w]
        euler = euler_from_quaternion(quat)
        self.odom_new.theta = euler[2]
        if not self.odom_ready:
            self.odom_ready = True
        #print(f"theta={self.odom_new.theta:.3f}")
        #print(f"x={self.current_x},y={self.current_y},theta={self.current_theta}")

def main(args=None):
    rclpy.init(args=args)
    move_commander_node = MoveCommanderNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(move_commander_node, executor)
    
    move_commander_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
