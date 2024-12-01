import rclpy
from rclpy import Node

class Explore(Node):

    def __init__(self):
        super().__init__('explore')
        self.tmr = self.create_timer(1/100, self.timer_callback)

    def timer_callback(self):
        pass

def main(args=None):
    """Entrypoint for the explore node."""
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
