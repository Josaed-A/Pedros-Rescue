import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__('ps4_teleop_node')


        self.axis_linear = 1 # Izquierdo en Y
        self.axis_angular = 0 # Izquierdo en X

        self.deadman_button = 4 # Trae a L1 como activador
        self.stop_button = 1 # Trae al circulo para detener

        # Zona muerta para evitar ruido
        self.deadzone = 0.008
        
        # Creamos el topico Joy 
        # Decimos que se ejecute joy_callback al recibir datos
        # Y tiene una cola de mensajes de 10

        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info('Nodo PS4 Teleop iniciado. Esperando mensajes en /joy....')

    def apply_deadzone(self, value):
        if abs(value)< self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        
        # Valores crudos sin aplicar la zona muerta
        raw_linear_value = msg.axes[self.axis_linear]
        raw_angular_value = msg.axes[self.axis_angular] * -1 

        #Valores con zona muerta
        linear_value = self.apply_deadzone(raw_linear_value)
        angular_value = self.apply_deadzone(raw_angular_value)


        deadman_pressed = msg.buttons[self.deadman_button]
        stop_pressed = msg.buttons[self.stop_button]

        self.get_logger().info(
            f'linear_axis={linear_value:.3f}, '
            f'raw_linear_axis={raw_linear_value:.3f}, '
            f'angular_axis={angular_value:.3f}, '
            f'raw_angular_axis={raw_angular_value:.3f}, '
            f'L1={deadman_pressed}, '
            f'circle={stop_pressed}'
        )

def main (args=None):
    rclpy.init(args=args)

    node = PS4TeleopNode()

    try: rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
