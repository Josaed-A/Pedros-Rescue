import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # =========================
        # Configuración general
        # =========================

        self.pin_factory = LGPIOFactory()

        # Límite físico de PWM.
        self.max_pwm = 1.0

        # Ganancias para balancear avance y giro.
        self.linear_gain = 1.0
        self.angular_gain = 0.35

        # Si un motor gira al revés, cambia su valor a -1.0.
        self.left_motor_direction = 1.0
        self.right_motor_direction = 1.0

        # Timeout de seguridad.
        # Si no llega /cmd_vel durante este tiempo, parada inmediata.
        self.cmd_timeout_seconds = 1.0
        self.last_cmd_time = self.get_clock().now()

        # =========================
        # Detección de estado
        # =========================

        # No es zona muerta del joystick.
        # Solo evita errores numéricos tipo 0.000000000000001.
        self.numeric_zero_epsilon = 1e-9

        # En curva, evita que una rueda se apague demasiado rápido.
        # 0.75 significa reducción máxima del 75% de la rueda interna.
        self.max_curve_reduction = 0.75

        # =========================
        # Perfil S adaptativo por diferencia
        # =========================

        # Objetivos calculados desde /cmd_vel.
        self.left_target = 0.0
        self.right_target = 0.0

        # Salida real suavizada que se manda al PWM.
        self.left_output = 0.0
        self.right_output = 0.0

        # Punto inicial de cada rampa.
        self.left_start_output = 0.0
        self.right_start_output = 0.0

        # Tiempo inicial de cada rampa.
        self.profile_start_time = self.now_seconds()

        # Tiempo actual de rampa.
        self.current_ramp_time = 0.10

        # Frecuencia del perfil.
        self.profile_period = 0.02  # 20 ms = 50 Hz

        # Cambios muy pequeños se aplican casi inmediatamente.
        # Esto NO bloquea el movimiento; solo evita que esos cambios tengan rampa lenta.
        self.immediate_delta = 0.04

        # Tiempo mínimo y máximo de rampa.
        self.min_ramp_time = 0.08
        self.max_ramp_time = 12.00

        # Si hay cambio de sentido, se protege con un tiempo mínimo.
        self.direction_change_min_ramp = 1.50

        # Si el cambio de target es menor que esto, no se reinicia la rampa.
        # No es zona muerta del joystick; es para evitar reiniciar la rampa por ruido numérico.
        self.target_update_epsilon = 0.002

        # Multiplicadores por estado.
        # Recto usa la rampa base.
        # Curva y giro responden algo más rápido.
        self.straight_ramp_factor = 5.00
        self.curve_ramp_factor = 0.55
        self.pivot_ramp_factor = 0.75
        self.stop_ramp_factor = 0.80

        self.last_motion_state = 'STOP'
        self.log_counter = 0

        # =========================
        # Pines GPIO - BTS7960
        # Numeración BCM
        # =========================

        # Motor izquierdo
        self.left_rpwm_pin = 12
        self.left_lpwm_pin = 13
        self.left_ren_pin = 27
        self.left_len_pin = 17

        # Motor derecho
        self.right_rpwm_pin = 18
        self.right_lpwm_pin = 19
        self.right_ren_pin = 22
        self.right_len_pin = 23

        # =========================
        # Salidas GPIO motor izquierdo
        # =========================

        self.left_rpwm = PWMOutputDevice(
            self.left_rpwm_pin,
            frequency=1000,
            initial_value=0.0,
            pin_factory=self.pin_factory
        )

        self.left_lpwm = PWMOutputDevice(
            self.left_lpwm_pin,
            frequency=1000,
            initial_value=0.0,
            pin_factory=self.pin_factory
        )

        self.left_ren = DigitalOutputDevice(
            self.left_ren_pin,
            initial_value=False,
            pin_factory=self.pin_factory
        )

        self.left_len = DigitalOutputDevice(
            self.left_len_pin,
            initial_value=False,
            pin_factory=self.pin_factory
        )

        # =========================
        # Salidas GPIO motor derecho
        # =========================

        self.right_rpwm = PWMOutputDevice(
            self.right_rpwm_pin,
            frequency=1000,
            initial_value=0.0,
            pin_factory=self.pin_factory
        )

        self.right_lpwm = PWMOutputDevice(
            self.right_lpwm_pin,
            frequency=1000,
            initial_value=0.0,
            pin_factory=self.pin_factory
        )

        self.right_ren = DigitalOutputDevice(
            self.right_ren_pin,
            initial_value=False,
            pin_factory=self.pin_factory
        )

        self.right_len = DigitalOutputDevice(
            self.right_len_pin,
            initial_value=False,
            pin_factory=self.pin_factory
        )

        # =========================
        # ROS 2 publishers/subscribers
        # =========================

        self.real_speed_publisher = self.create_publisher(
            Float32,
            '/real_speed_abs',
            10
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # =========================
        # Habilitar puentes H
        # =========================

        self.left_ren.on()
        self.left_len.on()

        self.right_ren.on()
        self.right_len.on()

        self.stop_all_motors()

        # =========================
        # Timers
        # =========================

        self.safety_timer = self.create_timer(
            0.1,
            self.safety_check
        )

        self.profile_timer = self.create_timer(
            self.profile_period,
            self.profile_timer_callback
        )

        self.get_logger().info('Nodo Motor Driver iniciado.')
        self.get_logger().info('Escuchando /cmd_vel...')
        self.get_logger().info('Publicando /real_speed_abs...')
        self.get_logger().info(f'PWM máximo físico: {self.max_pwm * 100:.0f}%')
        self.get_logger().info('Perfil S adaptativo por diferencia activo.')
        self.get_logger().info(f'Timeout de seguridad: {self.cmd_timeout_seconds:.2f} s')

    # =========================
    # Utilidades generales
    # =========================

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def clamp(self, value, min_value=-1.0, max_value=1.0):
        if value > max_value:
            return max_value

        if value < min_value:
            return min_value

        return value

    def is_zero(self, value):
        return abs(value) <= self.numeric_zero_epsilon

    def smootherstep(self, t):
        """
        Curva S:
        S(t) = 6t^5 - 15t^4 + 10t^3

        t debe estar entre 0 y 1.
        """

        t = self.clamp(t, 0.0, 1.0)

        return (6.0 * t**5) - (15.0 * t**4) + (10.0 * t**3)

    def interpolate_s_curve(self, start_value, target_value, elapsed_time, ramp_time):
        """
        Interpola desde start_value hasta target_value usando curva S.
        """

        if ramp_time <= 0.0:
            return target_value

        progress = elapsed_time / ramp_time
        progress = self.clamp(progress, 0.0, 1.0)

        s = self.smootherstep(progress)

        return start_value + (target_value - start_value) * s

    # =========================
    # Máquina de estados de movimiento
    # =========================

    def detect_motion_state(self, linear_x, angular_z):
        """
        Estados:
        STOP       -> no avance, no giro
        STRAIGHT   -> avance/retroceso recto
        PIVOT_TURN -> giro sobre el propio eje
        CURVE_TURN -> avance/retroceso con giro
        """

        linear_active = not self.is_zero(linear_x)
        angular_active = not self.is_zero(angular_z)

        if not linear_active and not angular_active:
            return 'STOP'

        if linear_active and not angular_active:
            return 'STRAIGHT'

        if not linear_active and angular_active:
            return 'PIVOT_TURN'

        return 'CURVE_TURN'

    def calculate_motor_targets(self, linear_x, angular_z):
        """
        Calcula left_target y right_target según el estado.
        """

        state = self.detect_motion_state(linear_x, angular_z)

        linear_x = self.clamp(linear_x)
        angular_z = self.clamp(angular_z)

        if state == 'STOP':
            left_motor = 0.0
            right_motor = 0.0

        elif state == 'STRAIGHT':
            left_motor = linear_x
            right_motor = linear_x

        elif state == 'PIVOT_TURN':
            # Giro sobre el propio eje.
            # Una rueda en un sentido y la otra en el contrario.
            left_motor = -angular_z
            right_motor = angular_z

        else:
            # Curva mientras avanza o retrocede.
            # No se aplica left = linear - angular porque puede apagar
            # una rueda demasiado pronto.
            turn_strength = min(abs(angular_z), 1.0)
            reduction = turn_strength * self.max_curve_reduction

            if angular_z > 0.0:
                # Curva hacia un lado:
                # se reduce la rueda izquierda.
                left_motor = linear_x * (1.0 - reduction)
                right_motor = linear_x
            else:
                # Curva hacia el otro lado:
                # se reduce la rueda derecha.
                left_motor = linear_x
                right_motor = linear_x * (1.0 - reduction)

        left_motor = self.clamp(left_motor)
        right_motor = self.clamp(right_motor)

        left_motor = left_motor * self.left_motor_direction
        right_motor = right_motor * self.right_motor_direction

        return left_motor, right_motor, state

    # =========================
    # Cálculo matemático de la rampa
    # =========================

    def has_direction_change(self, new_left_target, new_right_target):
        """
        Detecta si algún motor cambia de sentido.
        """

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
        """
        Ajusta la respuesta según el estado.
        La función matemática base es la misma, solo cambia el factor.
        """

        if state == 'PIVOT_TURN':
            return self.pivot_ramp_factor

        if state == 'CURVE_TURN':
            return self.curve_ramp_factor

        if state == 'STOP':
            return self.stop_ramp_factor

        return self.straight_ramp_factor

    def get_adaptive_ramp_time(self, new_left_target, new_right_target, state):
        """
        Calcula cuánto tarda en pasar de la salida actual
        al nuevo objetivo.

        delta pequeño  -> rampa corta o inmediata
        delta grande   -> rampa larga
        cambio sentido -> protección
        """

        delta_left = abs(new_left_target - self.left_output)
        delta_right = abs(new_right_target - self.right_output)

        delta = max(delta_left, delta_right)

        if delta <= self.immediate_delta:
            return 0.0

        normalized_delta = (
            delta - self.immediate_delta
        ) / (
            1.0 - self.immediate_delta
        )

        normalized_delta = self.clamp(normalized_delta, 0.0, 1.0)

        smooth_delta = self.smootherstep(normalized_delta)

        ramp_time = self.min_ramp_time + (
            self.max_ramp_time - self.min_ramp_time
        ) * smooth_delta

        ramp_time = ramp_time * self.get_state_ramp_factor(state)

        if self.has_direction_change(new_left_target, new_right_target):
            ramp_time = max(ramp_time, self.direction_change_min_ramp)

        return ramp_time

    def should_update_target(self, new_left_target, new_right_target, state):
        """
        Decide si se debe iniciar una nueva rampa.

        No bloquea movimientos pequeños desde cero.
        Solo evita reinicios por ruido numérico mientras ya hay un target casi igual.
        """

        if state != self.last_motion_state:
            return True

        left_diff = abs(new_left_target - self.left_target)
        right_diff = abs(new_right_target - self.right_target)

        if self.is_zero(self.left_target) and not self.is_zero(new_left_target):
            return True

        if self.is_zero(self.right_target) and not self.is_zero(new_right_target):
            return True

        if left_diff > self.target_update_epsilon:
            return True

        if right_diff > self.target_update_epsilon:
            return True

        return False

    def update_motor_targets(self, new_left_target, new_right_target, state):
        """
        Actualiza objetivos e inicia una nueva rampa S.
        """

        if not self.should_update_target(new_left_target, new_right_target, state):
            return

        now = self.now_seconds()

        ramp_time = self.get_adaptive_ramp_time(
            new_left_target,
            new_right_target,
            state
        )

        self.left_start_output = self.left_output
        self.right_start_output = self.right_output

        self.left_target = new_left_target
        self.right_target = new_right_target

        self.current_ramp_time = ramp_time
        self.profile_start_time = now

        self.last_motion_state = state

    # =========================
    # Publicación de velocidad real
    # =========================

    def get_real_speed_abs(self):
        """
        Velocidad real normalizada aplicada por la Raspberry.

        Se usa max(abs(left_output), abs(right_output)) porque si una rueda
        todavía se mueve, el robot no está completamente detenido.
        """

        return max(
            abs(self.left_output),
            abs(self.right_output)
        )

    def publish_real_speed(self):
        msg = Float32()
        msg.data = float(self.get_real_speed_abs())

        self.real_speed_publisher.publish(msg)

    # =========================
    # Control BTS7960
    # =========================

    def set_motor(self, rpwm, lpwm, power):
        """
        Envía potencia al BTS7960.

        power > 0 -> RPWM activo
        power < 0 -> LPWM activo
        power = 0 -> detenido
        """

        power = self.clamp(power, -1.0, 1.0)

        if abs(power) < 0.0001:
            rpwm.value = 0.0
            lpwm.value = 0.0
            return

        speed = abs(power) * self.max_pwm
        speed = self.clamp(speed, 0.0, 1.0)

        if power > 0.0:
            rpwm.value = speed
            lpwm.value = 0.0
        else:
            rpwm.value = 0.0
            lpwm.value = speed

    def stop_all_motors(self):
        """
        Parada inmediata.
        Se usa para timeout, apagado o fallo.
        """

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

    # =========================
    # Seguridad
    # =========================

    def safety_check(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_timeout_seconds:
            self.stop_all_motors()

    # =========================
    # Callback de /cmd_vel
    # =========================

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x * self.linear_gain
        angular_z = msg.angular.z * self.angular_gain

        left_motor, right_motor, state = self.calculate_motor_targets(
            linear_x,
            angular_z
        )

        # Aquí no se manda PWM.
        # Solo se actualizan los objetivos.
        self.update_motor_targets(
            left_motor,
            right_motor,
            state
        )

    # =========================
    # Timer del perfil S
    # =========================

    def profile_timer_callback(self):
        now = self.now_seconds()
        elapsed = now - self.profile_start_time

        self.left_output = self.interpolate_s_curve(
            self.left_start_output,
            self.left_target,
            elapsed,
            self.current_ramp_time
        )

        self.right_output = self.interpolate_s_curve(
            self.right_start_output,
            self.right_target,
            elapsed,
            self.current_ramp_time
        )

        self.left_output = self.clamp(self.left_output)
        self.right_output = self.clamp(self.right_output)

        self.set_motor(self.left_rpwm, self.left_lpwm, self.left_output)
        self.set_motor(self.right_rpwm, self.right_lpwm, self.right_output)

        self.publish_real_speed()

        self.log_counter += 1

        if self.log_counter >= 25:
            self.log_counter = 0

            left_pwm = abs(self.left_output) * self.max_pwm
            right_pwm = abs(self.right_output) * self.max_pwm
            real_speed = self.get_real_speed_abs()

            self.get_logger().info(
                f'perfil_s -> '
                f'state={self.last_motion_state}, '
                f'ramp={self.current_ramp_time:.2f}s, '
                f'real_speed={real_speed:.3f}, '
                f'left_target={self.left_target:.3f}, '
                f'left_output={self.left_output:.3f}, '
                f'left_pwm={left_pwm:.3f}, '
                f'right_target={self.right_target:.3f}, '
                f'right_output={self.right_output:.3f}, '
                f'right_pwm={right_pwm:.3f}'
            )

    # =========================
    # Cierre seguro
    # =========================

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