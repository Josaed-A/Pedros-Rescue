import glob
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

        self.declare_parameter('dev', 'auto')
        self.declare_parameter('device_name_contains', 'Elite 2')
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('autorepeat_rate', 20.0)

        self.dev_setting = str(self.get_parameter('dev').value)
        self.device_name_contains = str(
            self.get_parameter('device_name_contains').value
        )
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.autorepeat_rate = float(self.get_parameter('autorepeat_rate').value)

        self.publisher = self.create_publisher(Joy, 'joy', 10)
        self.axes = []
        self.buttons = []
        self.fd = None
        self.dev = None
        self.warned_missing = False
        self.changed = False

        self.poll_timer = self.create_timer(0.01, self.poll)
        repeat_period = 1.0 / self.autorepeat_rate if self.autorepeat_rate > 0.0 else 0.5
        self.repeat_timer = self.create_timer(repeat_period, self.publish)
        self.open_device()

    def resolve_device(self):
        if self.dev_setting != 'auto':
            return self.dev_setting

        candidates = []
        for sys_path in sorted(glob.glob('/sys/class/input/js*')):
            name_path = os.path.join(sys_path, 'device', 'name')
            try:
                with open(name_path, encoding='utf-8') as name_file:
                    name = name_file.read().strip()
            except OSError:
                continue
            candidates.append((name, f'/dev/input/{os.path.basename(sys_path)}'))

        preferred = self.device_name_contains.casefold()
        for name, path in candidates:
            if preferred and preferred in name.casefold():
                return path

        if preferred:
            return None
        return candidates[0][1] if candidates else None

    def open_device(self):
        if self.fd is not None:
            return True

        device = self.resolve_device()
        if device is None:
            if not self.warned_missing:
                self.get_logger().warn(
                    'No se encontro un joystick Linux que coincida con '
                    f'"{self.device_name_contains}"'
                )
                self.warned_missing = True
            return False

        try:
            self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
            self.dev = device
            self.axes = []
            self.buttons = []
            self.changed = False
            self.warned_missing = False
            self.get_logger().info(f'Joystick abierto en {device}')
            return True
        except OSError as exc:
            if not self.warned_missing:
                self.get_logger().warn(f'No se pudo abrir {device}: {exc}')
                self.warned_missing = True
            return False

    def close_device(self):
        if self.fd is not None:
            os.close(self.fd)
        self.fd = None
        self.dev = None
        self.axes = []
        self.buttons = []
        self.changed = False

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
                self.close_device()
                break

            if len(data) != EVENT_SIZE:
                self.get_logger().warn(f'Joystick desconectado: {self.dev}')
                self.close_device()
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
        # Never repeat an empty or stale state. The teleop watchdog will publish
        # zero after joy_timeout_seconds when the gamepad is disconnected.
        if self.fd is None or not self.axes or not self.buttons:
            return

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
        self.close_device()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
