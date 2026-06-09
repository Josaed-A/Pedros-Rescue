# Como ejecutar Pedro's Rescue

Esta guia asume que el repo ya esta en ambas maquinas y que ROS 2 esta instalado.

Reemplaza `<distro>` por tu version de ROS 2, por ejemplo `humble`, `jazzy` o la que esten usando.

## 1. Configurar red ROS 2

En la PC y en la Raspberry usa el mismo `ROS_DOMAIN_ID`:

```bash
export ROS_DOMAIN_ID=10
```

Si lo quieres dejar permanente, agregalo al `~/.bashrc` en ambas maquinas.

## 2. Compilar el workspace

Antes de compilar, instala los requisitos Python segun la maquina:

PC:

```bash
python3 -m pip install -r requirements_pc.txt
```

Raspberry:

```bash
python3 -m pip install -r requirements_raspberry.txt
```

Tambien revisa los comentarios dentro de cada archivo porque algunas dependencias de ROS 2 y hardware se instalan mejor con `apt`.

Ejecuta esto en la PC y en la Raspberry:

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Ejecutar en la Raspberry

La Raspberry controla motores y publica camaras.

### Terminal 1: motores

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_robot_core motor_driver_node
```

### Terminal 2: camara frontal Logitech

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_robot_core logitech_camera_node
```

Parametros utiles:

```bash
ros2 run rescue_robot_core logitech_camera_node --ros-args \
  -p index:=0 \
  -p width:=640 \
  -p height:=480 \
  -p fps:=30
```

### Terminal 3 opcional: Orbbec Astra RGB-D

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_robot_core astra_rgbd_camera_node
```

Parametros utiles:

```bash
ros2 run rescue_robot_core astra_rgbd_camera_node --ros-args \
  -p depth_index:=2 \
  -p color_index:=3 \
  -p fps:=30 \
  -p fx:=525.0 \
  -p fy:=525.0 \
  -p cx:=319.5 \
  -p cy:=239.5 \
  -p point_cloud_stride:=4
```

## 4. Ejecutar en la PC

### Terminal 1: control PS4

```bash
source /opt/ros/<distro>/setup.bash
ros2 run joy joy_node
```

### Terminal 2: teleoperacion

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_command_station ps4_teleop_node
```

### Terminal 3: dashboard con video en vivo y QR

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_command_station dashboard_node
```

El dashboard escucha por defecto:

```text
/robot/camera/front/image_raw
```

Puedes cambiar el topico de video asi:

```bash
ros2 run rescue_command_station dashboard_node --ros-args \
  -p camera_topic:=/robot/camera/astra/color/image_raw
```

### Terminal 4 opcional: visor RGB-D Astra

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_command_station rgbd_viewer_node
```

## 5. Controles

- Joystick izquierdo hacia adelante: ambas orugas avanzan.
- Joystick izquierdo hacia atras: ambas orugas retroceden.
- Joystick izquierdo hacia un lado: giro sobre el eje.
- Joystick izquierdo en diagonal: una oruga va mas rapido que la otra.
- `R1`: subir caja.
- `L1`: bajar caja.

## 6. Verificaciones utiles

Ver si el control publica:

```bash
ros2 topic echo /joy
```

Ver comandos de velocidad:

```bash
ros2 topic echo /cmd_vel
```

Ver velocidad real reportada por la Raspberry:

```bash
ros2 topic echo /real_speed_abs
```

Ver video frontal:

```bash
ros2 topic hz /robot/camera/front/image_raw
```

Ver profundidad Astra:

```bash
ros2 topic hz /robot/camera/astra/depth/image_raw
```

Ver nube de puntos Astra:

```bash
ros2 topic hz /robot/camera/astra/points
```

Ver nodos activos:

```bash
ros2 node list
```

Ver topicos activos:

```bash
ros2 topic list
```

## 7. Orden recomendado de arranque

1. Raspberry: `motor_driver_node`.
2. Raspberry: `logitech_camera_node`.
3. PC: `joy_node`.
4. PC: `ps4_teleop_node`.
5. PC: `dashboard_node`, si se quiere monitorear video, QR y manejo.
6. Opcional: `astra_rgbd_camera_node` y `rgbd_viewer_node`.

Si `/cmd_vel` deja de llegar a la Raspberry por mas de 1 segundo, el nodo de motores detiene las salidas por seguridad.

La Astra publica imagen de profundidad en `16UC1` y nube de puntos en `PointCloud2`.
Para mapeo 3D preciso hay que calibrar `fx`, `fy`, `cx`, `cy` y `depth_scale`.
