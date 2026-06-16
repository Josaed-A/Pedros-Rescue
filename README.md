# Pedro's Rescue

Plataforma ROS 2 para un robot tipo oruga de RoboCup Rescue, con **brazo 6-DOF**
(estabilización de cámara) y **patas** (4 servos de locomoción/postura).

El stack corre repartido en **dos máquinas**:

- **PC** (estación de mando): GUIs, SLAM, cinemática del brazo, control de patas.
- **Raspberry Pi** (núcleo del robot): drivers de hardware (motores, servos, cámaras, lidar).

## Paquetes

| Paquete | Máquina | Tipo | Qué hace |
|---|---|---|---|
| `rescue_interfaces` | — | ament_cmake | Mensajes y servicios custom (msg/srv). **Obligatorio aparte** (ROS 2 genera el código con rosidl, que un paquete Python no puede hacer). |
| `rescue_robot_core` | Pi | ament_python | TODO el hardware: motores (BTS7960), bus de servos Dynamixel (brazo AX-12A + patas) + EX-106+, cámaras. |
| `rescue_command_station` | PC | ament_python | Estación de mando: dashboard (conducción + **patas**), teleop PS4, y el **módulo `arm/`** (cinemática/cartesiano/GUI del brazo). |
| `rescue_bringup` | ambas | ament_python | Launch files de orquestación + nodos pegamento (SLAM, detección, geotiff, acumulador de nube). |
| `rescue_robot_description` | ambas | ament_python | URDF del robot (TF para RViz/SLAM). |
| `dependencias/` | — | varios | Reimplementaciones mínimas propias de terceros: `cv_bridge`, `joy`. Agrupado para no mezclarlo con los paquetes `rescue_*`. |

> El brazo NO es un paquete aparte: vive como **módulo `arm/` dentro de `rescue_command_station`** (corre en el PC, igual que el dashboard, así que no hay frontera de máquina que justifique separarlo). Las patas tampoco: su **driver** está en `rescue_robot_core` (comparten el bus AX-12A del brazo) y su **control/UI** en el dashboard.

## Estructura

```text
src/
  rescue_interfaces/              # msg/ + srv/ (rosidl)

  rescue_robot_core/              # Pi — hardware
    rescue_robot_core/
      camera_drivers/  # publicadores de cámaras
      config/          # pines, ganancias, perfil S
      drivers/         # BTS7960 (motores)
      motion/          # tracción diferencial + perfil S
      servos/          # wheel_encoder, params (Dynamixel)
      nodes/           # motor_driver_node, dynamixel_bus_node (brazo+patas),
                       #   ex106_driver_node, dynamixel_sim_node
    config/servos.yaml + launch/  (robot_core.launch.py, servos.launch.py)

  rescue_command_station/         # PC — estación de mando
    rescue_command_station/
      control/   # cajas y mezcla tipo tanque
      input/     # lectura del control PS4
      vision/    # QR y conversión de imagen para GUI
      arm/       # BRAZO: kinematics, cartesian_controller, *_node (cinematica/cartesian/gui/sim)
      nodes/     # dashboard_node (conducción + patas), ps4_teleop_node, rgbd_viewer_node
    config/arm.yaml + launch/  (command_station.launch.py, arm_station.launch.py)

  rescue_bringup/                 # launch + nodos pegamento
  rescue_robot_description/       # URDF

  dependencias/                   # terceros: cv_bridge, joy
```

## Flujo de control (conducción)

```text
PS4 / joy_node
    -> rescue_command_station / ps4_teleop_node   (joystick IZQUIERDO)
    -> /cmd_vel
    -> rescue_robot_core / motor_driver_node  -> perfil S -> BTS7960 -> motores
```

El robot publica `/real_speed_abs` para que el dashboard vea la velocidad real.

## Flujo del brazo 6-DOF

```text
GUI del brazo (rescue_command_station/arm/gui_node)   [se abre desde el dashboard]
    -> /compute_ik_pose, /cartesian/*   (cinematica_node, cartesian_node — PC)
    -> /ax12a/joint_cmd  +  /ex106/joint_cmd
    -> rescue_robot_core / dynamixel_bus_node + ex106_driver_node  (Pi)
    -> servos AX-12A (brazo) + EX-106+ (hombro)
    <- /arm/joint_states -> FK -> /end_effector_pose
```

- IK numérica por Jacobiano (damped least squares); la GUI tiene vista 3D + pinza.
- `/joint_states` del brazo se remapea a `/arm/joint_states` para no contaminar el `robot_state_publisher` de la base.

## Flujo de las patas (locomoción)

```text
PS4 / joy_node   (joystick DERECHO)
    -> rescue_command_station / dashboard_node  (panel PATAS)
    -> /legs/cmd   (eje X = par delantero, eje Y = par trasero; signo = dirección)
    -> rescue_robot_core / dynamixel_bus_node   (modo rueda, velocidad fija)
    -> 4 servos AX-12A de las patas (mismo bus que el brazo)
    <- /legs/state -> grados acumulados en vivo en el dashboard
```

- Botones por pata para habilitar/aislar, **calibrar (0°)** y **rehabilitar torque**.
- Las patas **solo** se mueven desde el dashboard (no desde la GUI del brazo).

## Flujo de visión

```text
rescue_robot_core / logitech_camera_node  -> /robot/camera/front/image_raw      -> dashboard (video + QR)
rescue_robot_core / astra_rgbd_camera_node -> /robot/camera/astra/{color,depth,points} -> dashboard + SLAM 3D
```

La Astra publica profundidad `16UC1` y nube `PointCloud2` (intrínsecos `fx,fy,cx,cy` configurables).

## Manejo tipo tanque (joystick izquierdo)

- Adelante/atrás: ambas orugas avanzan/retroceden. Lado: giran opuestas. Diagonal: una más rápida.
- `R1`/`L1`: subir/bajar caja. 5 cajas (20/40/60/80/100%). Sin acelerador R2 ni habilitación Share.

## Ejecución

Conducción + sensores:
```bash
# Raspberry
ros2 launch rescue_robot_core robot_core.launch.py     # motores + cámaras
ros2 launch rescue_bringup pi_sensors.launch.py        # lidar + Orbbec
# PC
ros2 launch rescue_command_station command_station.launch.py   # joy + teleop + dashboard
```

Brazo + patas (servos en el mismo bus AX-12A):
```bash
# Raspberry — bus de servos (brazo + patas)
ros2 launch rescue_robot_core servos.launch.py
# PC — el brazo se abre con el botón "CONTROL DEL BRAZO" del dashboard
#      (equivale a: ros2 launch rescue_command_station arm_station.launch.py)
# Patas: se controlan desde el panel PATAS del dashboard (stick derecho).
```

Con contenedores (ver [COMO_EJECUTAR.md](COMO_EJECUTAR.md)):
```bash
./scripts/run_slam_container.sh dashboard    # PC: dashboard (botón brazo, patas)
./scripts/run_slam_container.sh brazo-sim    # PC: brazo en SIMULACIÓN (sin la Pi)
./scripts/run_pi_sensors.sh servos           # Pi: bus de servos (brazo + patas)
```

## Requisitos

- PC: [requirements_pc.txt](requirements_pc.txt) (incluye customtkinter, matplotlib para la GUI del brazo)
- Raspberry: [requirements_raspberry.txt](requirements_raspberry.txt) (incluye dynamixel-sdk, pyserial)
- Apt/ROS: [system_requirements_pc.txt](system_requirements_pc.txt), [system_requirements_raspberry.txt](system_requirements_raspberry.txt)
- Reglas udev de hardware: [99-pedros-rescue.rules](99-pedros-rescue.rules)
