import os
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
EVENT_FORMAT = 'IhBB'
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)


class JoyNode(Node):
    def __init__(self):
        super().__init__('joy_node')

        self.declare_parameter('dev', '/dev/input/js0')
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('autorepeat_rate', 20.0)

        self.dev = self.get_parameter('dev').value
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.autorepeat_rate = float(self.get_parameter('autorepeat_rate').value)

        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.axes = []
        self.buttons = []
        self.fd = None
        self.warned_missing = False
        self.changed = False

        self.poll_timer = self.create_timer(0.01, self.poll)
        repeat_period = 1.0 / self.autorepeat_rate if self.autorepeat_rate > 0.0 else 0.5
        self.repeat_timer = self.create_timer(repeat_period, self.publish)
        self.open_device()

    def open_device(self):
        if self.fd is not None:
            return True

        try:
            self.fd = os.open(self.dev, os.O_RDONLY | os.O_NONBLOCK)
            self.warned_missing = False
            self.get_logger().info(f'Joystick abierto en {self.dev}')
            return True
        except OSError as exc:
            if not self.warned_missing:
                self.get_logger().warn(f'No se pudo abrir {self.dev}: {exc}')
                self.warned_missing = True
            return False

    def poll(self):
        if not self.open_device():
            return

        while True:
            try:
                data = os.read(self.fd, EVENT_SIZE)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().warn(f'Error leyendo {self.dev}: {exc}')
                os.close(self.fd)
                self.fd = None
                break

            if len(data) != EVENT_SIZE:
                break

            _time_ms, value, event_type, number = struct.unpack(EVENT_FORMAT, data)
            clean_type = event_type & ~JS_EVENT_INIT

            if clean_type == JS_EVENT_AXIS:
                self.ensure_len(self.axes, number + 1, 0.0)
                axis_value = max(-1.0, min(1.0, value / 32767.0))
                if abs(axis_value) < self.deadzone:
                    axis_value = 0.0
                self.axes[number] = axis_value
                self.changed = True
            elif clean_type == JS_EVENT_BUTTON:
                self.ensure_len(self.buttons, number + 1, 0)
                self.buttons[number] = 1 if value else 0
                self.changed = True

        if self.changed:
            self.publish()
            self.changed = False

    def publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self.axes)
        msg.buttons = list(self.buttons)
        self.publisher.publish(msg)

    @staticmethod
    def ensure_len(values, size, fill):
        while len(values) < size:
            values.append(fill)

    def destroy_node(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
