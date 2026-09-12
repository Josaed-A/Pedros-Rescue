# Como ejecutar Pedro's Rescue

Stack nativo probado: Raspberry Pi `gardian` + PC de mando.

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
- `joy_node` y `xbox_teleop_node` para el Xbox Elite 2.

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
ros2 launch rescue_bringup pedro_pc.launch.py joy_dev:=/dev/input/js0
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
- Xbox Elite 2: detectado en el PC como `Microsoft X-Box One Elite 2 pad` (`xpad`).
- Camaras: no se detectaron Orbbec ni Logitech por USB en la Pi; por eso el launch las omite en modo `auto`.
- AX-12A: falta `/dev/ax12a`; revisar adaptador USB/udev. `/dev/ex106` si existe.

## Puertos esperados

| Dispositivo | Puerto | Nota |
|---|---|---|
| LD19 | `/dev/ttyAMA0` | UART GPIO14/15, `230400` baud |
| AX-12A | `/dev/ax12a` | symlink udev del bus de brazo/patas |
| EX-106+ | `/dev/ex106` | symlink udev del servo hombro |
| Xbox Elite 2 | deteccion automatica (`/dev/input/js*`) | PC |
| Orbbec Astra | USB vendor `2bc5` | Pi |
| Logitech | USB vendor `046d` | Pi |

## Probar la deteccion HAZMAT

El nodo `object_detector` (AprilTag + hazmat YOLO + objetos de mision) corre
sobre la camara frontal (Logitech / GENERAL WEBCAM). Dos formas de probarlo:

### 1. Suelto, sin ROS (mas rapido para ajustar el modelo)

```bash
cd /home/semillero/Pedros-Rescue
python3 hazmat/training/test_hazmat_camera.py --device 2 --conf 0.40
```

`--device 2` es la GENERAL WEBCAM (confirmar con
`for d in /sys/class/video4linux/video*; do echo $d: $(cat $d/name); done`
si el indice cambia). Controles: `Q`/`ESC` salir, `+`/`-` ajustar confianza,
`S` guardar screenshot.

### 2. Dentro del stack completo (lo que corre en el robot real)

`pedro_pi.launch.py` ya lanza `object_detector` con `best.pt` por defecto en
cuanto detecta la Logitech por USB — no hace falta nada extra:

```bash
ssh gardian
cd ~/pedros
source scripts/ros_net_pi.sh
ros2 launch rescue_bringup pedro_pi.launch.py
```

Y en el PC, con `pedro_pc.launch.py` corriendo (ver arriba), el dashboard
muestra el feed frontal ya anotado (cajas de AprilTag/hazmat/objetos) en el
panel "Camara frontal". Verificar por CLI:

```bash
ros2 topic hz /camera/color/image_annotated/compressed
ros2 topic echo /object_detections
```

Si el panel frontal se queda en el video crudo (sin cajas), revisar en el log
de la Pi que `object_detector` haya cargado el YOLO ("YOLO (hazmat) cargado:
...") y no haya caido al fallback HSV — normalmente por `ultralytics` sin
instalar (`pip3 install -r requirements_raspberry.txt`) o por una ruta de
`hazmat_model` incorrecta (`hazmat_model:=/ruta/a/best.pt` para forzarla).

### 3. Prueba local con las dos camaras del PC (sin robot)

`ros2 launch rescue_bringup test_local_cameras.launch.py` lanza **dos**
instancias de `object_detector` (una por camara — GENERAL WEBCAM y la camara
local), ambas con deteccion activa al mismo tiempo, mas el dashboard con los
dos paneles ya anotados. Ver [hazmat/README.md](hazmat/README.md) y el
docstring de
[test_local_cameras.launch.py](src/rescue_bringup/launch/test_local_cameras.launch.py).

## Alertas hazmat (captura automatica para revision humana)

El modelo YOLO puede parpadear entre frames (falsos positivos, clase
inestable). Para no depender de que la deteccion se vea "sostenida" a simple
vista, `object_detector` la confirma solo cuando la misma señal aparece en
aprox. la misma zona durante `alert_confirm_frames` frames seguidos (default
3, ~1.5 s) y, al confirmarla, guarda automáticamente:

- `hazmat/alertas_detectadas/<camera_id>/<timestamp>_<clase>.jpg` — el frame
  anotado (con la caja) en el momento de la confirmación.
- `hazmat/alertas_detectadas/<camera_id>/<timestamp>_<clase>.json` — clase,
  confianza, bounding box y cámara de origen, para que una persona juzgue si
  la detección es correcta.

Hay cooldown (`alert_cooldown_sec`, default 20 s) para no llenar el disco
mientras la misma señal sigue en cuadro. Parámetros ajustables por launch:

```bash
ros2 launch rescue_bringup test_local_cameras.launch.py \
  alert_confirm_frames:=5 alert_cooldown_sec:=30.0
```

`alerts_dir` queda vacío por defecto en `pi_sensors.launch.py` /
`logitech_vision.launch.py` (sin captura automática en el robot real todavía);
pásalo explícitamente (`alerts_dir:=/ruta`) si se quiere activar ahí también.

## Guardar salidas de mision

Con el PC corriendo:

```bash
ros2 service call /save_geotiff std_srvs/srv/Trigger "{}"
ros2 service call /save_pointcloud std_srvs/srv/Trigger "{}"
```

Los archivos se guardan por defecto en `~/maps`.
