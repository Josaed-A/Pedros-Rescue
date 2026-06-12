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
      vision/    # QR y conversion de imagen para GUI
      nodes/     # nodos ROS 2 de PC
    launch/      # arranque completo de la estacion de mando

  rescue_robot_core/
    rescue_robot_core/
      camera_drivers/ # publicadores de camaras
      config/    # pines, ganancias, tiempos y limites
      drivers/   # salida a hardware BTS7960
      motion/    # traccion diferencial y perfil S
      nodes/     # nodos ROS 2 de la Raspberry
    launch/      # arranque completo del robot
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

## Flujo de vision

```text
Camaras en el robot
    -> rescue_robot_core / logitech_camera_node
    -> /robot/camera/front/image_raw
    -> rescue_command_station / dashboard_node
    -> video frontal en vivo + lector QR

Orbbec Astra en el robot
    -> rescue_robot_core / astra_rgbd_camera_node
    -> /robot/camera/astra/color/image_raw
    -> /robot/camera/astra/depth/image_raw
    -> /robot/camera/astra/points
    -> rescue_command_station / dashboard_node
    -> Astra color + profundidad + estado PointCloud2
```

La Astra publica imagen de profundidad `sensor_msgs/Image` con encoding `16UC1`
y una nube de puntos `sensor_msgs/PointCloud2` para usar despues en mapa 3D.
La nube usa intrinsecos configurables (`fx`, `fy`, `cx`, `cy`) y debe calibrarse
con los valores reales de la camara antes de usarla para mapeo preciso.

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
- `nodes/dashboard_node.py`: interfaz Tkinter para monitorear manejo, video en vivo y QR.
- `nodes/rgbd_viewer_node.py`: visor opcional para color + profundidad de la Astra.
- `launch/command_station.launch.py`: lanza joy, teleoperacion y GUI juntos.
- `vision/qr_detector.py`: deteccion y dibujo de codigos QR.
- `vision/tk_image.py`: conversion de frames OpenCV a imagenes Tkinter.

### `rescue_robot_core`

Corre en la Raspberry Pi y contiene:

- `config/motor_config.py`: pines GPIO, limites, ganancias y parametros del perfil S.
- `camera_drivers/logitech_camera_node.py`: publica la camara frontal en `/robot/camera/front/image_raw`.
- `camera_drivers/astra_rgbd_camera_node.py`: publica color, profundidad y nube de puntos de Astra.
- `camera_drivers/point_cloud.py`: convierte imagen de profundidad a `PointCloud2`.
- `motion/differential_drive.py`: mezcla diferencial para las dos orugas.
- `motion/s_curve.py`: funciones matematicas del suavizado.
- `drivers/bts7960.py`: escritura PWM al puente BTS7960.
- `nodes/motor_driver_node.py`: nodo ROS 2 que escucha `/cmd_vel` y controla motores.
- `launch/robot_core.launch.py`: lanza motores, Logitech y Astra juntos.

## Ejecucion

Arranque normal:

```bash
# Raspberry
ros2 launch rescue_robot_core robot_core.launch.py

# PC
ros2 launch rescue_command_station command_station.launch.py
```

Ver [COMO_EJECUTAR.md](COMO_EJECUTAR.md) para instalacion, parametros y diagnostico.

## Requisitos

- PC / estacion de mando: [requirements_pc.txt](requirements_pc.txt)
- Raspberry / nucleo del robot: [requirements_raspberry.txt](requirements_raspberry.txt)
- Apt/ROS PC: [system_requirements_pc.txt](system_requirements_pc.txt)
- Apt/ROS Raspberry: [system_requirements_raspberry.txt](system_requirements_raspberry.txt)
