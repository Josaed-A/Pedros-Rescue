# Como ejecutar Pedro's Rescue (RoboCup 2026)

Stack completo: Raspberry Pi 5 (sensores) + PC (SLAM + RViz + Dashboard).

---

## Arquitectura

```
Raspberry Pi 5  ─── ethernet ──→  PC Ubuntu
  10.42.0.240                      10.42.0.1

Pi publica:                     PC consume:
  /ldlidar_node/scan   ───────→  slam_toolbox
  /camera/depth/points ───────→  RViz (nube 3D)
  /camera/color/image_raw ───→  RViz (imagen)
  /camera/depth/image_raw ───→  RViz (profundidad)
```

---

## Paso 1 — Conexión ethernet PC ↔ Pi

PC IP: `10.42.0.1/24`  
Pi IP: `10.42.0.240/24` (fija, configurada en NetworkManager)

Verificar:
```bash
ping 10.42.0.240
```

---

## Paso 2 — Pi: lanzar sensores

SSH a la Pi y ejecutar en tmux:

```bash
ssh sraus@10.42.0.240     # pass: 123456
tmux new -s sensors        # o: tmux attach -t sensors
```

**Opción A — Nativo (recomendado, inmediato):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/pedros/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" multicast=\"false\"/></Interfaces></General><Discovery><Peers><Peer Address=\"10.42.0.1\"/></Peers></Discovery></Domain></CycloneDDS>"
ros2 launch rescue_bringup pi_sensors.launch.py
```

**Opción B — Contenedor (después de reconstruir imagen):**
```bash
chmod +x ~/pedros/scripts/run_pi_sensors.sh
~/pedros/scripts/run_pi_sensors.sh
```

**Opción C — Desde el PC via SSH (un solo comando):**
```bash
cd ~/Escritorio/PROYECTOS/Pedros-Rescue
./scripts/run_slam_container.sh pi
```

Verificar que los topics están publicando:
```bash
source /opt/ros/jazzy/setup.bash && source ~/pedros/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list | grep -E "scan|depth|color|point"
```
Debe aparecer:
- `/ldlidar_node/scan`
- `/camera/depth/points`
- `/camera/color/image_raw`
- `/camera/depth/image_raw`

---

## Paso 3 — PC: lanzar SLAM + RViz

```bash
cd ~/Escritorio/PROYECTOS/Pedros-Rescue
./scripts/run_slam_container.sh slam
```

Esto abre RViz con:
- Mapa SLAM construyéndose en tiempo real
- Nube de puntos 3D Orbbec (coloreada por altura)
- Scan del lidar LD19
- Modelo del robot

---

## Paso 4 — PC: lanzar Dashboard (opcional, ventana separada)

```bash
cd ~/Escritorio/PROYECTOS/Pedros-Rescue
./scripts/run_slam_container.sh dashboard
```

---

## Resumen comandos `run_slam_container.sh`

| Comando | Qué hace |
|---|---|
| `slam` | SLAM + RViz en PC (sin lidar/cámara) |
| `dashboard` | Ground station GUI en PC |
| `pi` | SSH → lanza sensores en Pi via contenedor |
| `pi-lidar` | SSH → solo lidar en Pi |
| `pi-camera` | SSH → solo cámara Orbbec en Pi |
| `pi-stop` | SSH → detiene contenedor de sensores |
| `pi-logs` | SSH → ver logs del contenedor Pi |
| `pi-build` | SSH → compila workspace en contenedor Pi |
| `build` | Compila workspace en contenedor PC |
| `rebuild-pi` | Reconstruye imagen Docker para Pi |
| `save-map` | Guarda mapa `.pgm` + `.yaml` |
| `save-mission` | Guarda GeoTIFF + PLY + CSV de misión |
| `topics` | Lista todos los topics activos |

---

## Resumen comandos `run_pi_sensors.sh` (en Pi)

| Comando | Qué hace |
|---|---|
| `./run_pi_sensors.sh` | Lidar + cámara Orbbec (completo) |
| `./run_pi_sensors.sh lidar` | Solo lidar LD19 |
| `./run_pi_sensors.sh camera` | Solo cámara Orbbec |
| `./run_pi_sensors.sh stop` | Detener contenedor |
| `./run_pi_sensors.sh logs` | Ver logs en vivo |
| `./run_pi_sensors.sh build` | Compilar workspace en contenedor |

---

## Hardware

| Dispositivo | Puerto Pi | Topic |
|---|---|---|
| Lidar LD19 | `/dev/ttyAMA0` (GPIO14/15, UART) | `/ldlidar_node/scan` |
| Orbbec Astra (depth) | USB (`2bc5:0403`) | `/camera/depth/points` |
| Orbbec Astra (color) | USB (`2bc5:0501`) | `/camera/color/image_raw` |

Pi config necesaria en `/boot/firmware/config.txt`:
```
enable_uart=1
dtoverlay=uart0-pi5
```

---

## Reconstruir imagen Pi (si Dockerfile.pi cambia)

```bash
# Desde el PC — copia y rebuilda en Pi
./scripts/run_slam_container.sh rebuild-pi
# ó desde SSH en Pi:
podman build -t localhost/pedros-rescue-ros2:jazzy -f ~/pedros/Dockerfile.pi ~/pedros/
```

---

## Topics principales

```text
/ldlidar_node/scan                    LaserScan — lidar LD19
/camera/depth/points                  PointCloud2 — nube 3D (sin color)
/camera/depth_registered/points       PointCloud2 — nube 3D con color RGB
/camera/color/image_raw               Image — RGB Orbbec
/camera/depth/image_raw               Image — profundidad 16-bit
/map                                   OccupancyGrid — mapa SLAM
/accumulated_pointcloud               PointCloud2 — nube acumulada misión
```

---

## Guardar mapa al final de la misión

```bash
./scripts/run_slam_container.sh save-mission   # GeoTIFF + PLY + CSV
./scripts/run_slam_container.sh save-map       # solo nav2 .pgm/.yaml
```

Los archivos se guardan en `~/maps/`.