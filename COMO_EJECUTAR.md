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

### Acceso SSH a la Pi (`gardian`)

Configuración actual para acceder a la Raspberry Pi del brazo de rescate **solo como usuario `gardian`**.

#### Config del cliente (PC, `~/.ssh/config`)

```
Host gardian-wifi
    HostName 192.168.38.137
    User gardian
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 20
    ServerAliveCountMax 3
    TCPKeepAlive yes

Host gardian-cable
    HostName 10.42.0.240
    User gardian
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
    ServerAliveInterval 20
    ServerAliveCountMax 3
    TCPKeepAlive yes

Host gardian pedro-rpi
    HostName gardian
    User gardian
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
    HostKeyAlias gardian
    ServerAliveInterval 20
    ServerAliveCountMax 3
    TCPKeepAlive yes
    # Prueba el cable (rapido, 2s); si responde, relaya por cable SIN timeout de
    # inactividad; si no, por wifi. Antes 'nc -w 2' en el relay cortaba la
    # conexion a los 2s de inactividad (de ahi el 'Broken pipe').
    ProxyCommand sh -c 'nc -z -w 2 10.42.0.240 22 2>/dev/null && exec nc 10.42.0.240 22 || exec nc 192.168.38.137 22'
```

**Puntos clave:**
- `User gardian` está fijado en cada bloque → `ssh gardian` siempre entra como ese usuario, sin importar el usuario local del PC.
- `IdentitiesOnly yes` + `IdentityFile ~/.ssh/id_ed25519` → solo se ofrece esa llave (no prueba otras llaves del agente), evita fallback a password si el agente tiene otras identidades.
- Alias `gardian` es el que hay que usar día a día: detecta solo por cable (10.42.0.240, eth0) vs WiFi (192.168.38.137) probando el puerto 22 con `nc -z -w 2` antes de conectar, y no corta por inactividad como pasaba antes.
- IPs: cable fija (`10.42.0.240` vía `60-eth0-static.yaml`), WiFi variable (`192.168.38.137` desde 2026-07-01, cambia si cambia la red).

#### Config del servidor (Pi) — **pendiente de verificar**

No pude conectar a la Pi en este momento (ni cable ni WiFi responden en el puerto 22) para leer `/etc/ssh/sshd_config` y confirmarlo en vivo. Lo que sabemos por trabajo previo:

- El usuario del sistema es `gardian`, miembro del grupo `dialout` (acceso a `/dev/ax12a`, `/dev/ex106`, `/dev/ldlidar`).
- El acceso hoy funciona por llave pública (`id_ed25519`), no hay indicios de que se use password.

Para dejar el acceso SSH **restringido solo a `gardian`** del lado servidor (si no está ya así), cuando la Pi esté disponible conviene verificar/aplicar en `/etc/ssh/sshd_config`:

```
AllowUsers gardian
PermitRootLogin no
PasswordAuthentication no
PubkeyAuthentication yes
```

y `sudo systemctl restart ssh` después. Puedo confirmarlo y aplicarlo apenas la Pi esté encendida y alcanzable — avísame.

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

## Deteccion de señales HAZMAT

Toda la documentacion de deteccion (arquitectura, como probarla con y sin ROS,
alertas automaticas, entrenamiento del modelo) vive en
[hazmat/README.md](hazmat/README.md).

## Problemas conocidos del entorno

### `arm_gui_node` moria con `cannot import name 'docstring' from 'matplotlib'`

Sintoma: al lanzar cualquier launch que levante el dashboard aparecia un
traceback grande y `arm_gui_node ... process has died`. El resto del sistema
(camaras, deteccion) seguia funcionando, pero el muro de rojo hacia parecer
que todo habia fallado.

Causa: conviven dos matplotlib — el de apt (`python3-matplotlib` 3.5.1) y el
de pip (3.10 en `~/.local`). El paquete de apt instala
`/usr/lib/python3/dist-packages/matplotlib-3.5.1-nspkg.pth`, que al arrancar
el interprete hace `sys.modules.setdefault('mpl_toolkits', <el de apt>)`. Es
decir, registra el modulo **antes de cualquier import**, asi que `mpl_toolkits`
siempre resolvia al de 3.5.1 — cuyo `mplot3d` pide `from matplotlib import
docstring`, eliminado en matplotlib >= 3.6. Ni `sys.path` ni reinstalar pip lo
cambiaban.

Solucion aplicada (sin sudo y sin tocar versiones): se gana la carrera del
`setdefault` registrando primero el `mpl_toolkits` de pip, aprovechando que el
user-site se procesa antes que `dist-packages`:

```bash
# 1. hacer que el mpl_toolkits de pip sea paquete regular
touch ~/.local/lib/python3.10/site-packages/mpl_toolkits/__init__.py

# 2. pre-registrarlo al arrancar el interprete
cat > ~/.local/lib/python3.10/site-packages/00-mpl_toolkits-prefer-pip.pth <<'EOF'
import sys, os, importlib.util, importlib.machinery; _sd = sys._getframe(1).f_locals['sitedir']; _mp = os.path.join(_sd, 'mpl_toolkits'); _sp = os.path.isdir(_mp) and importlib.machinery.PathFinder.find_spec('mpl_toolkits', [_sd]); _sp and sys.modules.setdefault('mpl_toolkits', importlib.util.module_from_spec(_sp))
EOF
```

Verificar: `python3 -c "from mpl_toolkits.mplot3d import Axes3D; print('OK')"`.
Revertir: borrar esos dos archivos.

## Guardar salidas de mision

Con el PC corriendo:

```bash
ros2 service call /save_geotiff std_srvs/srv/Trigger "{}"
ros2 service call /save_pointcloud std_srvs/srv/Trigger "{}"
```

Los archivos se guardan por defecto en `~/maps`.
