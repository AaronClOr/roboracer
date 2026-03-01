import rclpy
from rclpy.node import Node

class RacerNode(Node):
    def __init__(self):
        super().__init__('racer_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('RoboRacer Node has started!')

    def timer_callback(self):
        self.get_logger().info('Vroom vroom... racer is active.')

def main(args=None):
    rclpy.init(args=args)
    node = RacerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()