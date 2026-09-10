# Como ejecutar Pedro's Rescue

Stack nativo probado: Raspberry Pi `gardian` + PC de mando.

## Integración del brazo 6R

El brazo incorpora el motor Python entregado y la representación Canvas original.
Antes de arrancar esta versión, seguir [ARM_6R.md](src/rescue_command_station/ARM_6R.md)
para instalar Chromium, revisar las medidas locales conservadas y probar primero
`ros2 launch rescue_command_station arm_station.launch.py sim:=true`.
Esta integración aún no se ha validado en un grafo ROS ni en hardware; los
resultados están en [VALIDACION.md](src/rescue_command_station/docs/integracion_6r/VALIDACION.md).

## Red

```text
Pi por cable: 10.42.0.240  (eth0)
PC por cable: 10.42.0.1    (eno1)

Pi por WiFi: 192.168.231.137
PC por WiFi: 192.168.231.15
```

Los scripts `scripts/ros_net_pi.sh` y `scripts/ros_net_pc.sh` configuran CycloneDDS en unicast. Incluyen el peer remoto y la IP local para que los procesos de la misma maquina tambien se descubran cuando multicast esta deshabilitado.

Verificacion rapida desde el PC:

```bash
ping 10.42.0.240
ssh gardian
```

## Lanzar todo

### 1. Pi

```bash
ssh gardian
cd ~/pedros
source scripts/ros_net_pi.sh
ros2 launch rescue_bringup pedro_pi.launch.py
```

Este launch levanta:

- `robot_state_publisher`, `joint_state_publisher` y TF base.
- LD19 en `/dev/ttyAMA0`, componente `/ldlidar_node`.
- Motores BTS7960 y publicacion `/real_speed_abs`.
- Servos AX-12A y EX-106+.
- Orbbec/Logitech solo si se detectan por USB.

### 2. PC

En otra terminal:

```bash
cd /home/semillero/Pedros-Rescue
source scripts/ros_net_pc.sh
ros2 launch rescue_bringup pedro_pc.launch.py
```

Este launch levanta:

- `slam_toolbox` consumiendo `/ldlidar_node/scan`.
- RViz con el modelo, mapa, scan y nubes.
- Dashboard.
- `joy_node` y `ps4_teleop_node`.

## Argumentos utiles

Forzar red:

```bash
ros2 launch rescue_bringup pedro_pi.launch.py network:=cable
ros2 launch rescue_bringup pedro_pc.launch.py network:=cable
ros2 launch rescue_bringup pedro_pi.launch.py network:=wifi
ros2 launch rescue_bringup pedro_pc.launch.py network:=wifi
```

PC sin ventanas:

```bash
ros2 launch rescue_bringup pedro_pc.launch.py launch_rviz:=false
ros2 launch rescue_bringup pedro_pc.launch.py launch_dashboard:=false
```

Pi sin hardware opcional:

```bash
ros2 launch rescue_bringup pedro_pi.launch.py launch_camera:=false launch_logitech:=false
ros2 launch rescue_bringup pedro_pi.launch.py launch_lidar:=false
ros2 launch rescue_bringup pedro_pi.launch.py launch_motors:=false
ros2 launch rescue_bringup pedro_pi.launch.py launch_servos:=false
```

## Verificar

Desde el PC, con la Pi corriendo:

```bash
cd /home/semillero/Pedros-Rescue
source scripts/ros_net_pc.sh
export ROS_DISABLE_DAEMON=1
ros2 node list
ros2 topic list
ros2 lifecycle get /slam_toolbox
```

Esperado con hardware completo:

```text
/ldlidar_node/scan
/cmd_vel
/real_speed_abs
/arm/joint_states
/legs/state
/robot/camera/front/image_raw/compressed
/robot/camera/astra/color/image_raw/compressed
/robot/camera/astra/depth/image_raw/compressed
/robot/camera/astra/points
/map
```

## Estado de la ultima prueba

Validado:

- DDS PC-Pi por cable funciona.
- `pedro_pi.launch.py` levanta motores, TF y nodos de servos.
- El componente LD19 carga dentro de `/ldlidar_container`.
- `pedro_pc.launch.py` levanta RViz, dashboard, `joy_node`, teleop y `slam_toolbox`.
- RViz carga plugins con las librerias vendor de la instalacion Jazzy del PC.
- `slam_toolbox` configura y activa.

Bloqueos de hardware detectados:

- LD19: abre `/dev/ttyAMA0` a `230400`, pero falla con `LDLidar communication KO`. Revisar alimentacion, GND comun, TX/RX cruzados, UART habilitado y que el lidar este girando.
- PS4: no existe `/dev/input/js0`; conectar/emparejar el control.
- Camaras: no se detectaron Orbbec ni Logitech por USB en la Pi; por eso el launch las omite en modo `auto`.
- AX-12A: falta `/dev/ax12a`; revisar adaptador USB/udev. `/dev/ex106` si existe.

## Puertos esperados

| Dispositivo | Puerto | Nota |
|---|---|---|
| LD19 | `/dev/ttyAMA0` | UART GPIO14/15, `230400` baud |
| AX-12A | `/dev/ax12a` | symlink udev del bus de brazo/patas |
| EX-106+ | `/dev/ex106` | symlink udev del servo hombro |
| PS4 | `/dev/input/js0` | PC |
| Orbbec Astra | USB vendor `2bc5` | Pi |
| Logitech | USB vendor `046d` | Pi |

## Guardar salidas de mision

Con el PC corriendo:

```bash
ros2 service call /save_geotiff std_srvs/srv/Trigger "{}"
ros2 service call /save_pointcloud std_srvs/srv/Trigger "{}"
```

Los archivos se guardan por defecto en `~/maps`.
