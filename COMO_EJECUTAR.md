# Como ejecutar Pedro's Rescue

Esta guia asume que el repo ya esta en la PC y en la Raspberry, y que ROS 2 ya esta instalado.

Reemplaza `<distro>` por tu version de ROS 2, por ejemplo `humble` o `jazzy`.

## 1. Red ROS 2

En la PC y en la Raspberry usa el mismo `ROS_DOMAIN_ID`:

```bash
export ROS_DOMAIN_ID=10
```

Si lo quieres dejar permanente, agregalo al `~/.bashrc` en ambas maquinas.

## 2. Dependencias

### PC

Dependencias apt/ROS:

```bash
sudo apt install $(grep -vE '^\s*(#|$)' system_requirements_pc.txt | sed "s/<distro>/$ROS_DISTRO/g")
```

Dependencias pip:

```bash
python3 -m pip install -r requirements_pc.txt
```

### Raspberry

Dependencias apt/ROS/hardware:

```bash
sudo apt install $(grep -vE '^\s*(#|$)' system_requirements_raspberry.txt | sed "s/<distro>/$ROS_DISTRO/g")
```

Dependencias pip:

```bash
python3 -m pip install -r requirements_raspberry.txt
```

## 3. Compilar

Ejecuta esto en la PC y en la Raspberry:

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Arranque normal

La idea es lanzar todo junto con un launch por maquina.

### Raspberry: robot completo

Este launch inicia motores, camara frontal Logitech y Astra RGB-D con nube de puntos:

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch rescue_robot_core robot_core.launch.py
```

Parametros utiles:

```bash
ros2 launch rescue_robot_core robot_core.launch.py \
  logitech_index:=0 \
  astra_depth_index:=2 \
  astra_color_index:=3 \
  fx:=525.0 \
  fy:=525.0 \
  cx:=319.5 \
  cy:=239.5 \
  depth_scale:=0.001 \
  point_cloud_stride:=4
```

### PC: estacion de mando completa

Este launch inicia `joy_node`, teleoperacion y la GUI. La GUI siempre muestra:

- telemetria de manejo
- camara frontal con lector QR
- Astra color
- Astra profundidad
- estado de nube de puntos

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch rescue_command_station command_station.launch.py
```

Parametros utiles:

```bash
ros2 launch rescue_command_station command_station.launch.py \
  front_camera_topic:=/robot/camera/front/image_raw \
  astra_color_topic:=/robot/camera/astra/color/image_raw \
  astra_depth_topic:=/robot/camera/astra/depth/image_raw \
  point_cloud_topic:=/robot/camera/astra/points
```

## 5. Controles

- Joystick izquierdo hacia adelante: ambas orugas avanzan.
- Joystick izquierdo hacia atras: ambas orugas retroceden.
- Joystick izquierdo hacia un lado: giro sobre el eje.
- Joystick izquierdo en diagonal: una oruga va mas rapido que la otra.
- `R1`: subir caja.
- `L1`: bajar caja.

## 6. Topicos principales

Control:

```text
/joy
/cmd_vel
/drive_status
/real_speed_abs
```

Vision:

```text
/robot/camera/front/image_raw
/robot/camera/astra/color/image_raw
/robot/camera/astra/depth/image_raw
/robot/camera/astra/points
```

## 7. Verificaciones utiles

```bash
ros2 topic hz /robot/camera/front/image_raw
ros2 topic hz /robot/camera/astra/color/image_raw
ros2 topic hz /robot/camera/astra/depth/image_raw
ros2 topic hz /robot/camera/astra/points
ros2 topic echo /cmd_vel
ros2 topic echo /real_speed_abs
ros2 node list
ros2 topic list
```

## 8. Diagnostico por nodo

Usa estos comandos solo si necesitas probar una parte por separado.

Raspberry:

```bash
ros2 run rescue_robot_core motor_driver_node
ros2 run rescue_robot_core logitech_camera_node
ros2 run rescue_robot_core astra_rgbd_camera_node
```

PC:

```bash
ros2 run joy joy_node
ros2 run rescue_command_station ps4_teleop_node
ros2 run rescue_command_station dashboard_node
```

## 9. Nota de mapa 3D

La Astra publica profundidad `16UC1` y nube `PointCloud2`.
Para mapeo 3D preciso hay que calibrar `fx`, `fy`, `cx`, `cy` y `depth_scale` con los valores reales de la camara.

Si `/cmd_vel` deja de llegar a la Raspberry por mas de 1 segundo, el nodo de motores detiene las salidas por seguridad.
