import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String

from rescue_command_station.control.gearbox import Gearbox
from rescue_command_station.control.tank_drive import TankDriveMixer
from rescue_command_station.input.ps4_controller import PS4ControllerMapper


class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__('ps4_teleop_node')

        self.declare_parameter('cmd_publish_rate_hz', 20.0)
        self.declare_parameter('joy_timeout_seconds', 0.7)

        self.controller_mapper = PS4ControllerMapper()
        self.gearbox = Gearbox()
        self.drive_mixer = TankDriveMixer()

        self.cmd_publish_rate_hz = float(self.get_parameter('cmd_publish_rate_hz').value)
        self.joy_timeout_seconds = float(self.get_parameter('joy_timeout_seconds').value)

        self.real_speed_abs = 0.0
        self.last_controller_state = None
        self.last_joy_time = None
        self.last_cmd = Twist()
        self.last_target_speed = 0.0
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_left_track = 0.0
        self.last_right_track = 0.0
        self.last_status_text = 'CONTROL ACTIVO'
        self.log_counter = 0

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(Float32, '/real_speed_abs', self.real_speed_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.drive_status_publisher = self.create_publisher(String, '/drive_status', 10)

        cmd_period = 1.0 / max(self.cmd_publish_rate_hz, 1.0)
        self.create_timer(cmd_period, self.publish_cmd_heartbeat)
        self.create_timer(0.2, self.publish_periodic_status)

        self.get_logger().info('Estacion de mando iniciada.')
        self.get_logger().info('Joystick izquierdo: control tipo tanque.')
        self.get_logger().info('R1 sube caja, L1 baja caja.')
        self.get_logger().info(f'Heartbeat /cmd_vel: {self.cmd_publish_rate_hz:.1f} Hz')

    def real_speed_callback(self, msg):
        self.real_speed_abs = msg.data

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def has_fresh_joy(self):
        if self.last_joy_time is None:
            return False

        return (self.now_seconds() - self.last_joy_time) <= self.joy_timeout_seconds

    def set_zero_command(self, status_text='SIN JOYSTICK'):
        self.last_cmd = Twist()
        self.last_target_speed = 0.0
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_left_track = 0.0
        self.last_right_track = 0.0
        self.last_status_text = status_text

    def publish_cmd_heartbeat(self):
        if not self.has_fresh_joy():
            self.set_zero_command()

        self.cmd_vel_publisher.publish(self.last_cmd)

    def publish_drive_status(
        self,
        controller_state=None,
        target_speed=0.0,
        linear_x=0.0,
        angular_z=0.0,
        left_track=0.0,
        right_track=0.0,
        status_text='CONTROL ACTIVO'
    ):
        payload = {
            'gear': self.gearbox.current_gear,
            'gear_limit': float(self.gearbox.get_gear_limit()),
            'target_speed': float(target_speed),
            'real_speed_abs': float(self.real_speed_abs),
            'linear_x': float(linear_x),
            'angular_z': float(angular_z),
            'left_track': float(left_track),
            'right_track': float(right_track),
            'status_text': status_text,
        }

        if controller_state is not None:
            payload.update({
                'joy_x': float(controller_state.joystick_x),
                'joy_y': float(controller_state.joystick_y),
                'l1_pressed': int(controller_state.l1_pressed),
                'r1_pressed': int(controller_state.r1_pressed),
            })
        else:
            payload.update({
                'joy_x': 0.0,
                'joy_y': 0.0,
                'l1_pressed': 0,
                'r1_pressed': 0,
            })

        msg = String()
        msg.data = json.dumps(payload)
        self.drive_status_publisher.publish(msg)

    def publish_periodic_status(self):
        if not self.has_fresh_joy():
            self.set_zero_command()

        self.publish_drive_status(
            controller_state=self.last_controller_state,
            target_speed=self.last_target_speed,
            linear_x=self.last_linear_x,
            angular_z=self.last_angular_z,
            left_track=self.last_left_track,
            right_track=self.last_right_track,
            status_text=self.last_status_text,
        )

    def joy_callback(self, msg):
        controller_state = self.controller_mapper.from_joy_msg(msg)
        self.last_controller_state = controller_state
        self.last_joy_time = self.now_seconds()

        self.gearbox.update_from_controller(controller_state)

        drive_command = self.drive_mixer.build_command(controller_state, self.gearbox)

        cmd = Twist()
        cmd.linear.x = drive_command.linear_x
        cmd.angular.z = drive_command.angular_z
        self.last_cmd = cmd
        self.cmd_vel_publisher.publish(self.last_cmd)

        self.last_target_speed = drive_command.target_speed
        self.last_linear_x = cmd.linear.x
        self.last_angular_z = cmd.angular.z
        self.last_left_track = drive_command.left_track
        self.last_right_track = drive_command.right_track
        self.last_status_text = (
            'JOYSTICK EN REPOSO'
            if drive_command.target_speed == 0.0
            else 'CONTROL ACTIVO'
        )

        self.publish_periodic_status()
        self.log_status(controller_state, drive_command, cmd)

    def log_status(self, controller_state, drive_command, cmd):
        self.log_counter += 1

        if self.log_counter < 10:
            return

        self.log_counter = 0
        self.get_logger().info(
            f'gear={self.gearbox.current_gear}, '
            f'gear_limit={self.gearbox.get_gear_limit():.2f}, '
            f'joy_x={controller_state.joystick_x:.3f}, '
            f'joy_y={controller_state.joystick_y:.3f}, '
            f'left_track={drive_command.left_track:.3f}, '
            f'right_track={drive_command.right_track:.3f}, '
            f'linear.x={cmd.linear.x:.3f}, '
            f'angular.z={cmd.angular.z:.3f}, '
            f'real_speed_abs={self.real_speed_abs:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PS4TeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
