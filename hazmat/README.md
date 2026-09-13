# Detección de señales HAZMAT

Todo lo relacionado con la lectura de señales HAZMAT vive aquí: modelo,
entrenamiento, prueba standalone con OpenCV y las capturas de evidencia.

```text
hazmat/
├── README.md            → este documento (referencia completa de detección)
├── models/              → symlink a ../src/rescue_bringup/models
│                          best.pt (hazmat, 49 clases) + yolov8n.pt (COCO preentrenado)
├── training/
│   ├── train_hazmat.py       → entrenamiento sobre datasets/hazmat/
│   ├── export_to_onnx.py     → export .pt → .onnx
│   ├── test_hazmat_camera.py → prueba standalone (OpenCV puro, sin ROS)
│   └── runs/                 → checkpoints de entrenamiento (gitignored)
└── alertas_detectadas/  → capturas automáticas de evidencia (gitignored)
    ├── front/
    └── astra/
```

## Por qué `models/` es un symlink

`src/rescue_bringup/models/` es un recurso del paquete ROS `rescue_bringup`:
se instala vía `data_files` en `setup.py` y los launch files lo resuelven con
`get_package_share_directory('rescue_bringup')/models/`. Moverlo rompería el
empaquetado `ament_python`, así que el symlink lo deja visible junto al resto
del material hazmat sin duplicarlo ni sacarlo de su ubicación real.

Ahí viven **los dos modelos** del proyecto:

| Archivo | Qué es | En git |
|---|---|---|
| `best.pt` | Modelo hazmat entrenado (49 clases de señales) | Sí |
| `yolov8n.pt` | Preentrenado COCO de ultralytics (objetos de misión: persona/víctima, mochila, botella…) | No — se descarga solo |

`yolov8n.pt` está gitignored porque ultralytics lo descarga automáticamente a
esa misma ruta si falta; no hace falta versionarlo.

## Arquitectura

El nodo `object_detector` (paquete `rescue_bringup`) procesa cada cámara con
tres detectores independientes:

1. **AprilTag** `Standard41h12` (OpenCV `aruco`) → tipo `ar_code`
2. **Señales HAZMAT** con `best.pt` → tipo `hazmat_sign`
   (si el `.pt` falta o no hay `ultralytics`, cae a un detector HSV más tosco)
3. **Objetos de misión** con `yolov8n.pt` → tipo `real_object`

```text
cámara → logitech_pub (su propio cv2.VideoCapture)
       → /robot/camera/<cam>/image_raw/compressed
       → object_detector
            ├─ dibuja y publica el frame anotado EN CADA FRAME (~15 Hz)
            ├─ corre los detectores pesados cada `detect_interval_sec`
            └─ /object_detections · /object_detection_markers · CSV RoboCup
```

### Dos modos de hazmat

- **`hazmat_mode=local`** (default, lo que corre en el robot): cada
  `object_detector` carga su propio `best.pt` y lo ejecuta en línea.
- **`hazmat_mode=worker`**: el modelo se carga **una sola vez** en un nodo
  `hazmat_worker` compartido, que sirve a varias cámaras vía
  `/hazmat/submit/<camera_id>` → `/hazmat/result/<camera_id>`, con un
  scheduler de prioridad (round-robin normal; la cámara que confirma una
  señal sube de frecuencia y la otra baja a un piso de vigilancia, sin
  detenerse nunca).

Cada frame enviado al worker lleva un token de identidad
(`camera_id#frame_seq` + timestamp de captura) para que el resultado se
empareje **exactamente** con el frame que lo produjo. Si ese frame ya salió
del buffer cuando llega el resultado, se descarta — nunca se dibuja una
detección sobre un frame que no le corresponde.

### Fluidez del stream

El dibujo y la publicación del frame anotado ocurren en **cada** frame de
cámara, independientemente de la inferencia. Entre inferencias se repinta el
último recuadro conocido (`detection_hold_sec`), así el recuadro se ve continuo
en vez de parpadear. Parámetros relevantes:

| Parámetro | Default | Qué controla |
|---|---|---|
| `detect_interval_sec` | 0.5 | Cada cuánto corren los detectores pesados (no afecta el stream) |
| `detection_hold_sec` | 1.5 | Cuánto se sostiene el recuadro tras dejar de detectarse |
| `hazmat_imgsz` | 640 | Resolución de inferencia (416 = como el script standalone, más rápido) |
| `hazmat_submit_period_sec` | 0.5 | Cada cuánto se envía un frame al worker |

## Cómo probarlo

### 1. Standalone, sin ROS (lo más rápido para evaluar el modelo)

```bash
cd /home/semillero/Pedros-Rescue
python3 hazmat/training/test_hazmat_camera.py --device 2 --conf 0.40
```

`--device 2` es la GENERAL WEBCAM (confirmar el índice con
`for d in /sys/class/video4linux/video*; do echo $d: $(cat $d/name); done`).
Controles: `Q`/`ESC` salir, `+`/`-` ajustar confianza, `S` guardar captura.

### 2. Dentro del stack real del robot

`pedro_pi.launch.py` ya lanza `object_detector` con `best.pt` por defecto en
cuanto detecta la Logitech por USB:

```bash
ssh gardian
cd ~/pedros
source scripts/ros_net_pi.sh
ros2 launch rescue_bringup pedro_pi.launch.py
```

Con `pedro_pc.launch.py` corriendo en el PC, el dashboard muestra el feed
anotado en el panel "Camara frontal". Verificar por CLI:

```bash
ros2 topic hz /camera/color/image_annotated/compressed
ros2 topic echo /object_detections
```

Si el panel se queda en video crudo (sin cajas), revisar en el log de la Pi
que haya cargado el modelo (`YOLO (hazmat) cargado: ...`) y no haya caído al
fallback HSV — normalmente por `ultralytics` sin instalar
(`pip3 install -r setup/requirements_raspberry.txt`) o una ruta de
`hazmat_model` incorrecta.

### 3. Prueba local con las dos cámaras del PC (sin robot)

```bash
ros2 launch rescue_bringup test_local_cameras.launch.py
```

Levanta ambas cámaras del PC (cada una con su propio OpenCV), el
`hazmat_worker` compartido, un `object_detector` por cámara y el dashboard.
Es una herramienta de escritorio: **no toca** los launch de producción
(`pedro_pi`, `pedro_pc`, `pi_sensors`, `logitech_vision`), que siguen
apuntando a la Astra y la Logitech de la Pi.

Argumentos útiles:

```bash
# solo el modelo hazmat, sin AprilTag ni YOLO-objetos
ros2 launch rescue_bringup test_local_cameras.launch.py \
  enable_apriltag:=false enable_yolo:=false

# ajustar fluidez / frecuencia de inferencia
ros2 launch rescue_bringup test_local_cameras.launch.py \
  detect_interval_sec:=0.2 hazmat_imgsz:=416
```

## Alertas: captura automática para revisión humana

El modelo puede parpadear entre frames (falsos positivos, clase inestable).
Para no depender de eso, `object_detector` confirma una señal solo cuando
aparece en aproximadamente la misma zona durante `alert_confirm_frames`
inferencias seguidas, y al confirmarla guarda:

- `alertas_detectadas/<camera_id>/<timestamp>_<clase>.jpg` — el frame anotado
  exacto que produjo la detección.
- `alertas_detectadas/<camera_id>/<timestamp>_<clase>.json` — clase,
  confianza, bounding box y cámara de origen.

Hay cooldown (`alert_cooldown_sec`) para no llenar el disco mientras la misma
señal sigue en cuadro.

**Estado actual: suspendida.** `alerts_dir` está vacío por defecto en
`test_local_cameras.launch.py`, lo que deshabilita la captura sin tocar el
código. Para reactivarla:

```bash
ros2 launch rescue_bringup test_local_cameras.launch.py \
  alerts_dir:=/home/semillero/Pedros-Rescue/hazmat/alertas_detectadas
```

El contenido de `alertas_detectadas/` está gitignored (es evidencia de cada
corrida, no código); se conserva el `.gitkeep` para que la carpeta exista.

## Entrenamiento

```bash
python3 hazmat/training/train_hazmat.py --epochs 50 --model yolov8n.pt
python3 hazmat/training/export_to_onnx.py        # opcional, export a ONNX
```

El dataset se espera en `datasets/hazmat/data.yaml` (gitignored, >1 GB). El
modelo resultante se escribe en `src/rescue_bringup/models/best.pt`.

## Ver también

- [../README.md](../README.md) — visión general del proyecto.
- [../COMO_EJECUTAR.md](../COMO_EJECUTAR.md) — cómo levantar el robot completo.
