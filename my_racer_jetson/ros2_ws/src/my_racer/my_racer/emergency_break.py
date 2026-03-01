import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyBrake(Node):
    def __init__(self):
        super().__init__('safety_brake')
        # Subscribe to the LIDAR scan from the simulator
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        # Publisher to send drive commands back to the simulator
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def lidar_callback(self, data):
        # Get the distance to the object directly in front (middle of the scan array)
        center_index = len(data.ranges) // 2
        distance_front = data.ranges[center_index]

        drive_msg = AckermannDriveStamped()

        
        #drive_msg.drive.steering_angle_velocity = 0
        if distance_front < 1.0:
            self.get_logger().warn(f"WALL DETECTED! Distance: {distance_front:.2f}m. BRAKING!")
            drive_msg.drive.speed = 0.0  # Stop

        else:
            self.get_logger().info(f"Path Clear: {distance_front:.2f}m. Rolling...")
            drive_msg.drive.speed = 1.0  # Move forward slowly
            drive_msg.drive.steering_angle = 0.1
            
        self.publisher_.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyBrake()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()