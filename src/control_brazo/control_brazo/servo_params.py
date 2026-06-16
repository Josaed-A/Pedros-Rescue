"""
servo_params.py
===============
Lectura de la configuracion de servos desde parametros ROS.

ROS 2 no soporta listas de diccionarios en archivos YAML de parametros,
asi que servos.yaml usa mapas anidados (se aplanan a `servos.<nombre>.<campo>`)
mas una lista `servo_names` con el orden.
"""


def leer_servos_params(node) -> list[dict]:
    """Declara y lee servo_names + servos.<nombre>.*. Retorna lista de dicts."""
    node.declare_parameter('servo_names', [''])
    names = [n for n in node.get_parameter('servo_names').value if n]

    servos = []
    for n in names:
        node.declare_parameter(f'servos.{n}.id', 0)
        node.declare_parameter(f'servos.{n}.reduccion', 1.0)
        node.declare_parameter(f'servos.{n}.deadband_deg', 0.5)
        node.declare_parameter(f'servos.{n}.invertir_giro', False)
        servos.append({
            'nombre':        n,
            'id':            int(node.get_parameter(f'servos.{n}.id').value),
            'reduccion':     float(node.get_parameter(f'servos.{n}.reduccion').value),
            'deadband_deg':  float(node.get_parameter(f'servos.{n}.deadband_deg').value),
            'invertir_giro': bool(node.get_parameter(f'servos.{n}.invertir_giro').value),
        })
    return servos


def leer_wheels_params(node) -> list[dict]:
    """Declara y lee wheel_names + wheels.<nombre>.* (patas en modo rueda).

    Las patas giran en continuo (modo rueda) a velocidad fija; no usan
    deadband/posicion. Retorna lista de dicts con id, reduccion, invertir_giro.
    """
    node.declare_parameter('wheel_names', [''])
    names = [n for n in node.get_parameter('wheel_names').value if n]

    wheels = []
    for n in names:
        node.declare_parameter(f'wheels.{n}.id', 0)
        node.declare_parameter(f'wheels.{n}.reduccion', 1.0)
        node.declare_parameter(f'wheels.{n}.invertir_giro', False)
        wheels.append({
            'nombre':        n,
            'id':            int(node.get_parameter(f'wheels.{n}.id').value),
            'reduccion':     float(node.get_parameter(f'wheels.{n}.reduccion').value),
            'invertir_giro': bool(node.get_parameter(f'wheels.{n}.invertir_giro').value),
        })
    return wheels
