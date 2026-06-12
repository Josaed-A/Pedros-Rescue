from rescue_robot_core.motion.s_curve import clamp


def write_motor_pwm(rpwm, lpwm, power, max_pwm):
    power = clamp(power, -1.0, 1.0)

    if abs(power) < 0.0001:
        rpwm.value = 0.0
        lpwm.value = 0.0
        return

    speed = clamp(abs(power) * max_pwm, 0.0, 1.0)

    if power > 0.0:
        rpwm.value = speed
        lpwm.value = 0.0
    else:
        rpwm.value = 0.0
        lpwm.value = speed
