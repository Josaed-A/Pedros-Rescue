# Pedro's Rescue

Plataforma ROS 2 para un robot tipo oruga de RoboCup Rescue.

El repo esta organizado en dos paquetes grandes:

- `rescue_command_station`: estacion de mando que corre en la PC.
- `rescue_robot_core`: nucleo del robot que corre en la Raspberry Pi.

## Estructura

```text
src/
  rescue_command_station/
    rescue_command_station/
      control/   # cajas y mezcla tipo tanque
      input/     # lectura del control PS4
      nodes/     # nodos ROS 2 de PC

  rescue_robot_core/
    rescue_robot_core/
      config/    # pines, ganancias, tiempos y limites
      drivers/   # salida a hardware BTS7960
      motion/    # traccion diferencial y perfil S
      nodes/     # nodos ROS 2 de la Raspberry
```

## Flujo de control

```text
PS4 / joy_node
    -> rescue_command_station / ps4_teleop_node
    -> /cmd_vel
    -> rescue_robot_core / motor_driver_node
    -> perfil S
    -> drivers BTS7960
    -> motores
```

El robot publica `/real_speed_abs` para que la estacion de mando y el dashboard puedan ver la velocidad real aplicada.

## Manejo tipo tanque

Todo el movimiento sale del joystick izquierdo:

- Joystick hacia adelante: ambas orugas avanzan.
- Joystick hacia atras: ambas orugas retroceden.
- Joystick hacia un lado: las orugas giran en sentidos opuestos.
- Joystick diagonal: una oruga va mas rapido que la otra.

No hay acelerador con R2 y no hay habilitacion por Share.

## Cajas

Hay 5 cajas positivas:

- Caja 1: 20%
- Caja 2: 40%
- Caja 3: 60%
- Caja 4: 80%
- Caja 5: 100%

Controles:

- `R1`: subir caja.
- `L1`: bajar caja.

## Paquetes

### `rescue_command_station`

Corre en la PC y contiene:

- `input/ps4_controller.py`: traduce `sensor_msgs/Joy` a un estado simple del control.
- `control/gearbox.py`: maneja las 5 cajas.
- `control/tank_drive.py`: convierte joystick izquierdo a oruga izquierda/derecha y `Twist`.
- `nodes/ps4_teleop_node.py`: publica `/cmd_vel` y `/drive_status`.
- `nodes/dashboard_node.py`: interfaz Tkinter para monitorear el manejo.

### `rescue_robot_core`

Corre en la Raspberry Pi y contiene:

- `config/motor_config.py`: pines GPIO, limites, ganancias y parametros del perfil S.
- `motion/differential_drive.py`: mezcla diferencial para las dos orugas.
- `motion/s_curve.py`: funciones matematicas del suavizado.
- `drivers/bts7960.py`: escritura PWM al puente BTS7960.
- `nodes/motor_driver_node.py`: nodo ROS 2 que escucha `/cmd_vel` y controla motores.

## Ejecucion

Ver [COMO_EJECUTAR.md](COMO_EJECUTAR.md).
