import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from turtlesim.msg import Pose
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseArray, '/path_goal', self.goal_callback, 10)

        self.pose = None
        self.goal_queue = []
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(' TurtleController running...')

    def pose_callback(self, msg):
        self.pose = msg

    def goal_callback(self, msg):
        self.goal_queue = list(msg.poses)
        self.get_logger().info(f' Received {len(self.goal_queue)} goal(s)')

    def control_loop(self):
        if not self.pose or not self.goal_queue:
            return

        target = self.goal_queue[0]

        dx = target.position.x - self.pose.x
        dy = target.position.y - self.pose.y
        distance = math.hypot(dx, dy)

        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - self.pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # normalize angle

        twist = Twist()

        if abs(angle_diff) > 0.1:
            # Rotate first
            twist.angular.z = 2.0 * angle_diff
            twist.linear.x = 0.0
        elif distance > 0.1:
            # Move forward if angle is good
            twist.linear.x = 1.5 * distance
            twist.angular.z = 0.0
        else:
            self.get_logger().info(f'Reached ({target.position.x:.2f}, {target.position.y:.2f})')
            self.goal_queue.pop(0)

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
