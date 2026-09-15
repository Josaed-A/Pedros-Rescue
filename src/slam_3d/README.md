# SLAM 3D — Orbbec Astra Pro + RTAB-Map

Mapeo 3D en vivo para Pedro's Rescue. La Raspberry Pi publica color y
profundidad; el PC corre RTAB-Map y construye el mapa.

```
Raspberry Pi                          PC
─────────────                         ──
openni2_camera  → /depth_raw/image  ─┐
v4l2_camera     → /color/image_raw  ─┼→ rgbd_odometry → rtabmap → mapa 3D
                                     │
                          [ red WiFi / ethernet ]
```

Lo que viaja por la red son **dos imágenes 2D**, no la nube de puntos. La nube
la calcula RTAB-Map en el PC a partir de la profundidad y los intrínsecos. Un
`PointCloud2` de 640×480 con XYZ+RGB pesa ~5.9 MB por frame contra ~1.2 MB de la
imagen de profundidad: cinco veces más tráfico para la misma información.

---

## Requisitos

- ROS 2 Jazzy en ambas máquinas
- Orbbec Astra Pro (`lsusb` debe mostrar `2bc5:0403` **y** `2bc5:0501`)
- Ambas máquinas en la misma subred

## Instalación

En **las dos** máquinas:

```bash
bash scripts/install.sh
```

Instala RTAB-Map, los drivers de cámara, Cyclone DDS y las reglas udev.
Después, **desconecta y reconecta la cámara** para que apliquen los permisos.

### Verificar que la cámara responde

```bash
export LD_PRELOAD=$PWD/openni2_redist/arm64/libOpenNI2.so   # x64 en el PC
ros2 run openni2_camera list_devices
```

Debe imprimir el URI y el número de serie. Si dice `Found 0 devices`, ver
[Problemas conocidos](#problemas-conocidos).

## Red

En cada máquina, apuntando a la IP de la otra:

```bash
# En la Pi
source scripts/setup_env.sh <IP_DEL_PC> wlan0
# En el PC
source scripts/setup_env.sh <IP_DE_LA_PI> wlp1s0
```

Comprueba que el PC ve a la Pi:

```bash
ros2 topic list | grep depth_raw
```

## Ejecución

**Raspberry Pi:**
```bash
ros2 launch slam_3d slam3d_pi.launch.py
```

**PC:**
```bash
ros2 launch slam_3d slam3d_pc.launch.py
```

Antes de dar por bueno el arranque, comprueba que los **tres** nodos están
vivos:

```bash
ros2 node list | grep rtabmap
# /rtabmap/rgbd_odometry
# /rtabmap/rtabmap        <-- si falta este, el mapa NO se construye
# /rtabmap/rtabmap_viz
```

## Guardar el mapa

**Durante el recorrido:** pausa con el botón de la barra de herramientas y usa
`File → Export 3D clouds`. La opción solo se habilita con el mapeo pausado.

**Al terminar:** `Ctrl+C` en la terminal de RTAB-Map y espera a ver
`Saving database/long-term memory...done!`. Ese mensaje es la confirmación; si
matas el proceso con `kill -9` se pierde el mapa.

```bash
cp ~/.ros/rtabmap.db ~/mapa_$(date +%Y%m%d).db
rtabmap-export --cloud --output ~/mapa ~/mapa_$(date +%Y%m%d).db
```

Para revisar un mapa guardado: `rtabmap-databaseViewer ~/mapa_*.db`

---

## Problemas conocidos

Cada uno de estos costó horas de depuración. Están en orden de cuánto cuesta
encontrarlos.

### `Found 0 devices` con la cámara conectada

La causa es una incompatibilidad de librerías que **falla en silencio**.
`liborbbec.so` está compilado contra la libOpenNI2 de Orbbec (2.3.x), y la que
trae Ubuntu es la 2.2.0.33. OpenNI2 intenta cargar el driver, falla y reporta
cero dispositivos sin decir por qué. Parece un problema de hardware y no lo es.

Se resuelve forzando la libOpenNI2 de Orbbec con `LD_PRELOAD`. El launch ya lo
hace vía `additional_env`.

### `quality=0` siempre, con cientos de features extraídas

El `camera_info` del color viene con `k: [0,0,0,0,0,0,0,0,0]` porque la cámara
no está calibrada y `v4l2_camera` no tiene de dónde sacar los intrínsecos. Sin
`fx`, `fy`, `cx`, `cy` no se puede proyectar ningún píxel a 3D.

Por eso el launch usa `camera_info_topic:=/depth_raw/camera_info`: el driver
OpenNI2 sí rellena esos valores leyéndolos del dispositivo.

> **Pendiente:** esos intrínsecos son del sensor IR, no de la cámara RGB. Son
> lentes distintos. Funciona porque son parecidos, pero introduce error métrico.
> La solución de fondo es calibrar con `camera_calibration` y un tablero de
> ajedrez, y guardar el YAML en `~/.ros/camera_info/`.

### El nodo `rtabmap` no aparece en `ros2 node list`

Una base de datos creada por otra versión de RTAB-Map hace que el nodo aborte al
abrirla. El síntoma despista: `rgbd_odometry` funciona con buena calidad, la GUI
muestra la nube del frame actual, pero el mapa nunca se acumula ni se guarda.

```bash
rm -f ~/.ros/rtabmap.db
```

### `rtabmap` corre pero nunca procesa

Se suscribe a cinco tópicos con sincronización aproximada y necesita los cinco
alineados. Sobre red, con el delay variando, prácticamente nunca alinean.

Por eso el launch usa `subscribe_odom_info:=false` (pasa de 5 a 4 tópicos) y
`queue_size:=100` (el default de 10 es muy corto).

### La nube no se acumula, solo se ve lo que la cámara tiene enfrente

La odometría se pierde y reinicia. Cada reinicio arranca un mapa nuevo
desconectado del anterior, así que el historial queda fragmentado.

No es un frame malo lo que rompe: es la bola de nieve. Un fallo hace que la
odometría prediga, el siguiente falla y extrapola sobre esa predicción, y en 4-5
iteraciones las `guess` llegan a metros de salto. Ahí aparece
`All projected points are outside the camera` y ya no se recupera.

Mitiga, no resuelve: movimientos lentos, escenas con textura a 1-2 m, buena luz.
La solución real sería fusionar con odometría de ruedas o un IMU, que el robot
no tiene: el TF `odom → base_footprint` es un transform **estático identidad**.

### `compressedDepth` corrompe la profundidad

Comprimir el **color** en JPEG funciona perfecto y ahorra 8× de ancho de banda.
Comprimir la **profundidad** con `compressedDepth` rompe la odometría: mismo
pipeline, cambiando solo ese tópico, `quality` pasa de 150-230 a 0 permanente.

Color comprimido, profundidad cruda en 16 bits.

### Frames descartados y odometría rota sin causa aparente

Con batería baja el gobernador de CPU reduce la frecuencia, el `update time` de
RTAB-Map sube, se descartan frames y la odometría se rompe. **Ten el PC
enchufado a la corriente.**

Lo mismo si subes `Grid/DepthDecimation` a 1: el 100% de los píxeles satura el
PC y aparecen `Dropping image/scan data`. El valor 2 es el punto medio.

---

## Ancho de banda

Medido con `ros2 topic bw` a 640×480 y 30 fps:

| Tópico | Por frame | Mbps |
|---|---|---|
| `/depth/image` (32FC1, metros) | 1.23 MB | 176 |
| `/depth_raw/image` (16UC1, mm) | 0.61 MB | 44 |
| `/color/image_raw` (rgb8) | 0.92 MB | 58 |
| `/color/image_raw/compressed` (JPEG) | 0.11 MB | 28 |
| `/depth_raw/image/compressedDepth` (PNG16) | 0.027 MB | 6.4 |

La profundidad comprime 23× contra 8× del color: es una imagen suave, con
grandes regiones de valores similares, que es justo lo que PNG aprovecha. Pero
**no se puede usar** por lo del apartado anterior.

Configuración actual: **color crudo + profundidad 16 bits = ~102 Mbps**. Usando
color comprimido baja a ~72 Mbps.

### Sobre WiFi

Medido con `iperf3` en el mismo punto y con la misma configuración:

| Momento | Ancho de banda | Retransmisiones |
|---|---|---|
| Noche | 92 Mbps | 5 |
| Mañana siguiente | 15.8 Mbps | 181 |

Un hotspot de celular fluctúa demasiado para streaming RGB-D. **Mide siempre
antes de una demo:**

```bash
# En la Pi
iperf3 -s
# En el PC
iperf3 -c <IP_DE_LA_PI> -t 10
```

Regla práctica: no pasar del 60-70% del enlace. Por encima de eso la degradación
no es lineal (con 234 Mbps sobre un enlace de 92 no llegaba el 39% proporcional,
llegaba el 26%).

Si el enlace queda corto, en orden de cuánto ahorran:

- Bajar la profundidad a 15 fps → ahorra 22 Mbps, RTAB-Map va bien a 15 Hz
- Correr la detección de objetos en la Pi y mandar solo resultados → 28 Mbps
- Bajar la profundidad a 320×240 → de 44 a 11 Mbps
- `topic_tools throttle` para limitar la tasa sin tocar el driver

**Con cable ethernet nada de esto hace falta.** 1000 Mbps estables contra 92
inestables: diez veces más margen del necesario.

---

## Notas de hardware

- La Astra Pro necesita **USB 2.0 dedicado**. Si el LiDAR y el Arduino comparten
  el mismo controlador, la cámara pierde frames. En la Pi 5, usar un puerto
  USB 3.0 (azul) le da bus propio.
- **Nunca uses `/dev/videoN`** en la configuración. El número cambia entre
  máquinas: la Astra es `video0` en la Pi y `video2` en un portátil con webcam
  integrada. Usa siempre el path `/dev/v4l/by-id/...`.
- Antes de relanzar los drivers, siempre `pkill`. Un nodo que murió mal deja el
  dispositivo tomado y el error es `Resource busy` o
  `Failed mapping device memory`.
