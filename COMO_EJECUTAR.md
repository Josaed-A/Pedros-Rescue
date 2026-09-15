# Como ejecutar Pedro's Rescue

Stack nativo probado: Raspberry Pi `gardian` + PC de mando.

## Red usada en la sesión del 15 de septiembre de 2026

```text
PC (wlan0):  10.230.234.1
Pi (wlan0):  10.230.234.137
SSH estable: gardian@gardian.local
```

Estas direcciones se pasan al launch de forma explícita; no se modificaron los
defaults del código. Para esta red, los comandos exactos son:

```bash
# Raspberry Pi
source /opt/ros/jazzy/setup.bash
source ~/pedros/install/setup.bash
ros2 launch rescue_bringup pedro_pi.launch.py \
  network:=wifi wifi_interface:=wlan0 \
  pc_wifi_ip:=10.230.234.1 pi_wifi_ip:=10.230.234.137
```

```bash
# PC, desde la raíz de este repositorio
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rescue_bringup pedro_pc.launch.py \
  network:=wifi wifi_interface:=wlan0 \
  pi_wifi_ip:=10.230.234.137 pc_wifi_ip:=10.230.234.1
```

### Si vuelve a cambiar la IP

Primero consultar las direcciones, sin adivinarlas:

```bash
# PC
ip -4 -brief address
getent hosts gardian.local

# Pi (entrando por su nombre, aunque cambie de IP)
ssh gardian@gardian.local
ip -4 -brief address
```

Después sustituir las dos direcciones en **ambos** comandos anteriores. La
regla es simétrica:

| Lado | IP del otro equipo | IP propia |
|---|---|---|
| Pi | `pc_wifi_ip` | `pi_wifi_ip` |
| PC | `pi_wifi_ip` | `pc_wifi_ip` |

Los defaults permanentes están declarados en
`src/rescue_bringup/launch/pedro_pi.launch.py` y
`src/rescue_bringup/launch/pedro_pc.launch.py`, en los argumentos
`pc_wifi_ip`, `pi_wifi_ip` y `wifi_interface`. Los scripts alternativos
`scripts/ros_net_pi.sh` y `scripts/ros_net_pc.sh` también contienen `_PEER`,
`_SELF` e interfaz con valores fijos. Si otro agente decide cambiar defaults,
debe actualizar los dos lados a la vez. Para un cambio temporal basta pasar los
argumentos al launch como arriba; no es necesario editar código.

## Integración del brazo 6R

El brazo incorpora el motor Python entregado y la representación Canvas original.
Antes de arrancar esta versión, seguir [ARM_6R.md](src/rescue_command_station/ARM_6R.md)
para instalar Chromium, revisar las medidas locales conservadas y probar primero
`ros2 launch rescue_command_station arm_station.launch.py sim:=true`.
Esta integración aún no se ha validado en un grafo ROS ni en hardware; los
resultados están en [VALIDACION.md](src/rescue_command_station/docs/integracion_6r/VALIDACION.md).

## Defaults históricos de red

Los siguientes valores siguen en los scripts y launches como defaults, pero
**no corresponden a la red activa del 15 de septiembre de 2026**:

```text
Pi por cable: 10.42.0.240  (eth0)
PC por cable: 10.42.0.1    (eno1)

Pi por WiFi: 192.168.231.137
PC por WiFi: 192.168.231.15
```

Los scripts `scripts/ros_net_pi.sh` y `scripts/ros_net_pc.sh` configuran CycloneDDS en unicast. Incluyen el peer remoto y la IP local para que los procesos de la misma maquina tambien se descubran cuando multicast esta deshabilitado.

Verificación rápida desde el PC usando el nombre estable de la Pi:

```bash
getent hosts gardian.local
ping gardian.local
ssh gardian@gardian.local
```

## Lanzar todo

### 1. Pi

```bash
ssh gardian@gardian.local
cd ~/pedros
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rescue_bringup pedro_pi.launch.py \
  network:=wifi wifi_interface:=wlan0 \
  pc_wifi_ip:=10.230.234.1 pi_wifi_ip:=10.230.234.137
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
cd /home/sebastian/Documents/pedros-rescue
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rescue_bringup pedro_pc.launch.py \
  network:=wifi wifi_interface:=wlan0 \
  pi_wifi_ip:=10.230.234.137 pc_wifi_ip:=10.230.234.1
```

Este launch levanta:

- `slam_toolbox` consumiendo `/ldlidar_node/scan`.
- RViz con el modelo, mapa, scan y nubes.
- Dashboard.
- `joy_node` y `ps4_teleop_node`.

## Argumentos utiles

Al forzar `network`, pasar también interfaz y las dos IP si no coinciden con
los defaults históricos.

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
