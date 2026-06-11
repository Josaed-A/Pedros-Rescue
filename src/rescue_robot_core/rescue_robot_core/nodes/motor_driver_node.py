import rclpy
from geometry_msgs.msg import Twist
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from rclpy.node import Node
from std_msgs.msg import Float32

from rescue_robot_core.config import motor_config as cfg
from rescue_robot_core.drivers.bts7960 import write_motor_pwm
from rescue_robot_core.motion import s_curve
from rescue_robot_core.motion.differential_drive import calculate_motor_targets


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        self.declare_parameter('max_pwm', cfg.MAX_PWM)
        self.declare_parameter('pwm_frequency_hz', cfg.PWM_FREQUENCY_HZ)
        self.declare_parameter('cmd_timeout_seconds', cfg.CMD_TIMEOUT_SECONDS)

        self.max_pwm = float(self.get_parameter('max_pwm').value)
        self.pwm_frequency_hz = int(self.get_parameter('pwm_frequency_hz').value)
        self.cmd_timeout_seconds = float(self.get_parameter('cmd_timeout_seconds').value)

        self.pin_factory = LGPIOFactory()

        self.left_target = 0.0
        self.right_target = 0.0
        self.left_output = 0.0
        self.right_output = 0.0
        self.left_start_output = 0.0
        self.right_start_output = 0.0
        self.profile_start_time = self.now_seconds()
        self.current_ramp_time = 0.10
        self.last_motion_state = 'STOP'
        self.last_cmd_time = self.get_clock().now()
        self.log_counter = 0

        self.configure_gpio()

        self.real_speed_publisher = self.create_publisher(Float32, '/real_speed_abs', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_timer(0.1, self.safety_check)
        self.create_timer(cfg.PROFILE_PERIOD, self.profile_timer_callback)

        self.enable_bridges()
        self.stop_all_motors()

        self.get_logger().info('Nucleo del robot iniciado.')
        self.get_logger().info('Escuchando /cmd_vel...')
        self.get_logger().info('Publicando /real_speed_abs...')
        self.get_logger().info('Perfil S adaptativo activo.')
        self.get_logger().info(f'Timeout de seguridad: {self.cmd_timeout_seconds:.2f} s')
        self.get_logger().info(
            f'PWM maximo: {self.max_pwm:.2f}, frecuencia: {self.pwm_frequency_hz} Hz'
        )

    def configure_gpio(self):
        self.left_rpwm = PWMOutputDevice(
            cfg.LEFT_RPWM_PIN,
            frequency=self.pwm_frequency_hz,
            initial_value=0.0,
            pin_factory=self.pin_factory
        )
        self.left_lpwm = PWMOutputDevice(
            cfg.LEFT_LPWM_PIN,
            frequency=self.pwm_frequency_hz,
            initial_value=0.0,
            pin_factory=self.pin_factory
        )
        self.left_ren = DigitalOutputDevice(
            cfg.LEFT_REN_PIN,
            initial_value=False,
            pin_factory=self.pin_factory
        )
        self.left_len = DigitalOutputDevice(
            cfg.LEFT_LEN_PIN,
            initial_value=False,
            pin_factory=self.pin_factory
        )

        self.right_rpwm = PWMOutputDevice(
            cfg.RIGHT_RPWM_PIN,
            frequency=self.pwm_frequency_hz,
            initial_value=0.0,
            pin_factory=self.pin_factory
        )
        self.right_lpwm = PWMOutputDevice(
            cfg.RIGHT_LPWM_PIN,
            frequency=self.pwm_frequency_hz,
            initial_value=0.0,
            pin_factory=self.pin_factory
        )
        self.right_ren = DigitalOutputDevice(
            cfg.RIGHT_REN_PIN,
            initial_value=False,
            pin_factory=self.pin_factory
        )
        self.right_len = DigitalOutputDevice(
            cfg.RIGHT_LEN_PIN,
            initial_value=False,
            pin_factory=self.pin_factory
        )

    def enable_bridges(self):
        self.left_ren.on()
        self.left_len.on()
        self.right_ren.on()
        self.right_len.on()

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def is_zero(self, value):
        return abs(value) <= cfg.NUMERIC_ZERO_EPSILON

    def has_direction_change(self, new_left_target, new_right_target):
        left_change = (
            self.left_output * new_left_target < 0.0 and
            not self.is_zero(self.left_output) and
            not self.is_zero(new_left_target)
        )
        right_change = (
            self.right_output * new_right_target < 0.0 and
            not self.is_zero(self.right_output) and
            not self.is_zero(new_right_target)
        )

        return left_change or right_change

    def get_state_ramp_factor(self, state):
        if state == 'PIVOT_TURN':
            return cfg.PIVOT_RAMP_FACTOR

        if state == 'CURVE_TURN':
            return cfg.CURVE_RAMP_FACTOR

        if state == 'STOP':
            return cfg.STOP_RAMP_FACTOR

        return cfg.STRAIGHT_RAMP_FACTOR

    def get_adaptive_ramp_time(self, new_left_target, new_right_target, state):
        delta_left = abs(new_left_target - self.left_output)
        delta_right = abs(new_right_target - self.right_output)
        delta = max(delta_left, delta_right)

        if delta <= cfg.IMMEDIATE_DELTA:
            return 0.0

        normalized_delta = (
            delta - cfg.IMMEDIATE_DELTA
        ) / (
            1.0 - cfg.IMMEDIATE_DELTA
        )
        normalized_delta = s_curve.clamp(normalized_delta, 0.0, 1.0)
        smooth_delta = s_curve.smootherstep(normalized_delta)

        ramp_time = cfg.MIN_RAMP_TIME + (
            cfg.MAX_RAMP_TIME - cfg.MIN_RAMP_TIME
        ) * smooth_delta
        ramp_time = ramp_time * self.get_state_ramp_factor(state)

        if self.has_direction_change(new_left_target, new_right_target):
            ramp_time = max(ramp_time, cfg.DIRECTION_CHANGE_MIN_RAMP)

        return ramp_time

    def should_update_target(self, new_left_target, new_right_target, state):
        if state != self.last_motion_state:
            return True

        left_diff = abs(new_left_target - self.left_target)
        right_diff = abs(new_right_target - self.right_target)

        if self.is_zero(self.left_target) and not self.is_zero(new_left_target):
            return True

        if self.is_zero(self.right_target) and not self.is_zero(new_right_target):
            return True

        return (
            left_diff > cfg.TARGET_UPDATE_EPSILON or
            right_diff > cfg.TARGET_UPDATE_EPSILON
        )

    def update_motor_targets(self, new_left_target, new_right_target, state):
        if not self.should_update_target(new_left_target, new_right_target, state):
            return

        self.left_start_output = self.left_output
        self.right_start_output = self.right_output
        self.left_target = new_left_target
        self.right_target = new_right_target
        self.current_ramp_time = self.get_adaptive_ramp_time(
            new_left_target,
            new_right_target,
            state
        )
        self.profile_start_time = self.now_seconds()
        self.last_motion_state = state

    def get_real_speed_abs(self):
        return max(abs(self.left_output), abs(self.right_output))

    def publish_real_speed(self):
        msg = Float32()
        msg.data = float(self.get_real_speed_abs())
        self.real_speed_publisher.publish(msg)

    def set_motor(self, rpwm, lpwm, power):
        write_motor_pwm(rpwm, lpwm, power, self.max_pwm)

    def stop_all_motors(self):
        self.left_target = 0.0
        self.right_target = 0.0
        self.left_output = 0.0
        self.right_output = 0.0
        self.left_start_output = 0.0
        self.right_start_output = 0.0
        self.current_ramp_time = 0.0
        self.profile_start_time = self.now_seconds()
        self.last_motion_state = 'STOP'

        self.left_rpwm.value = 0.0
        self.left_lpwm.value = 0.0
        self.right_rpwm.value = 0.0
        self.right_lpwm.value = 0.0
        self.publish_real_speed()

    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_timeout_seconds:
            self.stop_all_motors()

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x * cfg.LINEAR_GAIN
        angular_z = msg.angular.z * cfg.ANGULAR_GAIN
        left_motor, right_motor, state = calculate_motor_targets(
            linear_x,
            angular_z,
            cfg.LEFT_MOTOR_DIRECTION,
            cfg.RIGHT_MOTOR_DIRECTION,
            cfg.NUMERIC_ZERO_EPSILON
        )

        self.update_motor_targets(left_motor, right_motor, state)

    def profile_timer_callback(self):
        elapsed = self.now_seconds() - self.profile_start_time

        self.left_output = s_curve.interpolate(
            self.left_start_output,
            self.left_target,
            elapsed,
            self.current_ramp_time
        )
        self.right_output = s_curve.interpolate(
            self.right_start_output,
            self.right_target,
            elapsed,
            self.current_ramp_time
        )

        self.left_output = s_curve.clamp(self.left_output)
        self.right_output = s_curve.clamp(self.right_output)

        self.set_motor(self.left_rpwm, self.left_lpwm, self.left_output)
        self.set_motor(self.right_rpwm, self.right_lpwm, self.right_output)
        self.publish_real_speed()
        self.log_profile_status()

    def log_profile_status(self):
        self.log_counter += 1

        if self.log_counter < 25:
            return

        self.log_counter = 0
        left_pwm = abs(self.left_output) * self.max_pwm
        right_pwm = abs(self.right_output) * self.max_pwm

        self.get_logger().info(
            f'perfil_s -> '
            f'state={self.last_motion_state}, '
            f'ramp={self.current_ramp_time:.2f}s, '
            f'real_speed={self.get_real_speed_abs():.3f}, '
            f'left_target={self.left_target:.3f}, '
            f'left_output={self.left_output:.3f}, '
            f'left_pwm={left_pwm:.3f}, '
            f'right_target={self.right_target:.3f}, '
            f'right_output={self.right_output:.3f}, '
            f'right_pwm={right_pwm:.3f}'
        )

    def shutdown_motors(self):
        self.get_logger().info('Apagando motores...')
        self.stop_all_motors()

        self.left_ren.off()
        self.left_len.off()
        self.right_ren.off()
        self.right_len.off()

        self.left_rpwm.close()
        self.left_lpwm.close()
        self.left_ren.close()
        self.left_len.close()
        self.right_rpwm.close()
        self.right_lpwm.close()
        self.right_ren.close()
        self.right_len.close()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_motors()
        node.destroy_node()
        rclpy.shutdown()
