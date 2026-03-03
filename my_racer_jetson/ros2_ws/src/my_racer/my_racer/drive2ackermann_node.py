import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def listener_callback(self, msg):
        new_msg = AckermannDriveStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.drive.speed = msg.linear.x
        # We treat angular velocity as steering angle for teleop
        new_msg.drive.steering_angle = msg.angular.z 
        self.publisher.publish(new_msg)

def main():
    rclpy.init()
    node = TwistToAckermann()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()