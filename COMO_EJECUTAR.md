# Como ejecutar Pedro's Rescue

Esta guia asume que el repo ya esta en la PC y en la Raspberry, y que ROS 2 ya esta instalado.

Reemplaza `<distro>` por tu version de ROS 2, por ejemplo `humble` o `jazzy`.

## 1. Red ROS 2

El WiFi que usa el robot no entrega multicast de forma confiable, asi que el
descubrimiento normal de DDS NO funciona entre la PC y la Raspberry. Se usa
un Fast DDS Discovery Server corriendo en la Raspberry.

En la Raspberry (ya configurado en crontab `@reboot`):

```bash
fastdds discovery -i 0 -l 0.0.0.0 -p 11811
```

En la PC y en la Raspberry (ya agregado al `~/.bashrc` de ambas):

```bash
export ROS_DISCOVERY_SERVER=<IP_DE_LA_RASPBERRY>:11811
```

IMPORTANTE: si la IP de la Raspberry cambia (es DHCP), hay que actualizar
esta variable en el `~/.bashrc` de las dos maquinas. IP actual: `192.168.231.137`.

Para que los comandos `ros2 topic list/echo/hz` vean todos los topicos en
modo discovery server, usa ademas:

```bash
export ROS_SUPER_CLIENT=TRUE
ros2 daemon stop   # reinicia el daemon con las variables nuevas
```

Nota: `ros2 topic echo/hz` por defecto se suscriben RELIABLE; las camaras
publican BEST_EFFORT. Para probar a mano agrega `--qos-reliability best_effort`.

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

## 3. Permisos USB para ROS

Ejecuta esto una vez en la PC y/o Raspberry que use hardware USB
como Arduino, control, camaras o adaptadores seriales:

```bash
cd ~/Pedros-Rescue
sudo scripts/habilitar_usb_ros.sh
```

Despues cierra sesion y vuelve a entrar, o reinicia la maquina, para que los
grupos nuevos apliquen. Verifica con:

```bash
id
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/input/js* /dev/video*
```

Grupos usados:

- `dialout`: puertos seriales `/dev/ttyUSB*` y `/dev/ttyACM*`.
- `video`: camaras `/dev/video*`.
- `input`: joystick/control `/dev/input/js*`.
- `plugdev`: dispositivos USB con reglas `udev` que usan este grupo.

## 4. Compilar

Ejecuta esto en la PC y en la Raspberry:

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Arranque normal

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
  astra_color_index:=2 \
  astra_depth_index:=-1 \
  jpeg_quality:=80
```

Nota: la Astra Pro solo expone su camara RGB por V4L2 (`/dev/video2`).
La profundidad real requiere el driver OpenNI2 de Orbbec, por eso
`astra_depth_index` queda en `-1` (deshabilitada) por defecto.

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
  front_camera_topic:=/robot/camera/front/image_raw/compressed \
  astra_color_topic:=/robot/camera/astra/color/image_raw/compressed \
  astra_depth_topic:=/robot/camera/astra/depth/image_raw/compressed \
  point_cloud_topic:=/robot/camera/astra/points
```

## 6. Controles

- Joystick izquierdo hacia adelante: ambas orugas avanzan.
- Joystick izquierdo hacia atras: ambas orugas retroceden.
- Joystick izquierdo hacia un lado: giro sobre el eje.
- Joystick izquierdo en diagonal: una oruga va mas rapido que la otra.
- `R1`: subir caja.
- `L1`: bajar caja.

## 7. Topicos principales

Control:

```text
/joy
/cmd_vel
/drive_status
/real_speed_abs
```

Vision (las imagenes viajan comprimidas como `sensor_msgs/CompressedImage`
para no saturar el WiFi; color en JPEG y profundidad en PNG 16 bits):

```text
/robot/camera/front/image_raw/compressed
/robot/camera/astra/color/image_raw/compressed
/robot/camera/astra/depth/image_raw/compressed
/robot/camera/astra/points
```

## 8. Verificaciones utiles

```bash
ros2 topic hz /robot/camera/front/image_raw/compressed
ros2 topic hz /robot/camera/astra/color/image_raw/compressed
ros2 topic hz /robot/camera/astra/depth/image_raw/compressed
ros2 topic hz /robot/camera/astra/points
ros2 topic bw /robot/camera/front/image_raw/compressed
ros2 topic echo /cmd_vel
ros2 topic echo /real_speed_abs
ros2 node list
ros2 topic list
```

## 9. Diagnostico por nodo

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

## 10. Nota de mapa 3D

La Astra publica profundidad `16UC1` y nube `PointCloud2`.
Para mapeo 3D preciso hay que calibrar `fx`, `fy`, `cx`, `cy` y `depth_scale` con los valores reales de la camara.

Si `/cmd_vel` deja de llegar a la Raspberry por mas de 1 segundo, el nodo de motores detiene las salidas por seguridad.
