import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import math

goal = [[0.0, 0.0, 0.0],
        [0.6, 0.0, 0.0],
        [0.6, 0.0, 1.57],
        [0.6, 0.6, 1.57]]
#goal_x = 0.0
#goal_y = 0.0
#goal_theta = 1.57

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class MoveCommanderNode(Node):
    def __init__(self):
        super().__init__('move_commander')
        self.sub_odom = self.create_subscription(Odometry,'/odom', self.new_odom, 10 )
        self.pub_cmdVel = self.create_publisher(Twist,
            '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_count = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.seq = 1

    def check_target(self):
        v = 0.0
        w = 0.0
        x1 = goal[self.seq][0]
        y1 = goal[self.seq][1]
        theta1 = goal[self.seq][2]

        x0 = goal[self.seq-1][0]
        y0 = goal[self.seq-1][1]
        theta0 = goal[self.seq-1][2]

        delta_x = x1 - x0
        delta_y = y1 - y0
        delta_theta = theta1 - theta0

        error_x = x1 - self.current_x
        error_y = y1 - self.current_y
        error_theta = theta1 - self.current_theta
        
        if abs(delta_theta) > 0.0:
            if abs(error_theta) < 0.05:
                self.seq += 1
                v = 0.0
                w = 0.0
                return v, w
            elif error_theta > 0.01:
                v = 0.0
                w = 0.3
                return v, w
            elif error_theta < -0.0:
                v = 0.0
                w = -0.3
                return v, w
        if abs(delta_x) > 0.0:
            if abs(error_x) < 0.01:
                self.seq += 1
                v = 0.0
                w = 0.0
                return v,w
            else:
                v = 0.2
                w = 0.0
                return v,w
        if abs(delta_y) > 0.0:
            if abs(error_y) < 0.01:
                self.seq += 1
                v = 0.0
                w = 0.0
                return v,w
            else:
                v = 0.2
                w = 0.0
                return v,w

    def timer_callback(self):
        #print(f"Timer Count={self.timer_count}")
        [vx, vy] = self.check_target()
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = vy
        self.pub_cmdVel.publish(cmd_vel)
        self.timer_count+=1
        #self.get_logger().info('Publishing: "%s"' % msg.data)

    def new_odom(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        [roll_x, pitch_y, yaw_z] = euler_from_quaternion(qx, qy, qz, qw)
        self.current_theta = yaw_z
        #print(f"theta={self.current_theta:.3f}")
        #print(f"x={self.current_x},y={self.current_y},theta={self.current_theta}")

def main(args=None):
    rclpy.init(args=args)
    move_commander_node = MoveCommanderNode()
    rclpy.spin(move_commander_node)
    move_commander_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
