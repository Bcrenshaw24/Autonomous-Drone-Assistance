import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')
        self.goal = [5.0, 5.0, 2.0]  # Example goal (x, y, z)
        self.kp = 0.5

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
    
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        x_dist = self.goal[0] - x
        y_dist = self.goal[1] - y
        z_dist = self.goal[2] - z

        distance = math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)

        if distance < 0.1:
            vel_msg = Twist()
            self.cmd_pub.publish(vel_msg)
            self.get_logger().info('Goal reached!')
            return
        
        vel_msg = Twist()
        vel_msg.linear.x = self.kp * x_dist
        vel_msg.linear.y = self.kp * y_dist
        vel_msg.linear.z = self.kp * z_dist
        self.cmd_pub.publish(vel_msg)
        self.get_logger().info(f'Navigating to goal: {self.goal}, Current position: ({x}, {y}, {z})')
        print(f'Distance to goal: {distance}')

def main(args=None):
    rclpy.init(args=args)
    goal_navigator = GoalNavigator()
    rclpy.spin(goal_navigator)
    goal_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
