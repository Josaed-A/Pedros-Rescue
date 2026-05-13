import rclpy
from rclpy.node import Node

class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__('ps4_teleop_node')

        self.get_logger().info('Nodo PS4 Teleop iniciado')

def main (args=None):
    rclpy.init(args=args)

    node = PS4TeleopNode()

    try: rclpy.spin(node)
    except KeyBoardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
