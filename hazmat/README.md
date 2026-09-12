# Hazmat — deteccion de señales

Todo lo relacionado con la deteccion de señales HAZMAT vive organizado aqui:

```text
hazmat/
├── models/              → symlink a ../src/rescue_bringup/models (best.pt + metadata Edge Impulse)
├── training/            → entrenamiento, export ONNX y prueba standalone con OpenCV
│   ├── train_hazmat.py
│   ├── export_to_onnx.py
│   ├── test_hazmat_camera.py
│   └── runs/            → checkpoints de entrenamiento (gitignored)
└── alertas_detectadas/  → capturas automaticas cuando object_detector confirma una señal
    ├── front/           → capturas de la camara frontal (Logitech / GENERAL WEBCAM)
    └── astra/           → capturas de la camara Astra (o su sustituto local de prueba)
```

## Por que `models/` es un symlink

`src/rescue_bringup/models/best.pt` es un recurso del paquete ROS `rescue_bringup`
(se instala via `data_files` en `setup.py` y varios launch files lo referencian
con `get_package_share_directory('rescue_bringup')/models/best.pt`). No se puede
mover sin romper el empaquetado `ament_python` ni todos esos defaults, asi que
en vez de duplicarlo aqui hay un symlink para que quede visible/organizado
junto al resto de material hazmat sin moverlo de su ubicacion real.

## `alertas_detectadas/`

`object_detector.py` (paquete `rescue_bringup`) guarda aqui automaticamente una
captura (frame anotado + JSON con la descripcion de la señal) cada vez que una
deteccion hazmat se **confirma** — es decir, se mantiene estable durante varios
frames seguidos, no un parpadeo de un solo frame — para dejar a juicio humano
revisar si la deteccion es correcta. Ver `alert_confirm_frames`,
`alert_cooldown_sec` y `alerts_dir` en `object_detector.py` /
`test_local_cameras.launch.py`.

Contenido gitignored (son evidencias de cada corrida, no código); se conserva
`alertas_detectadas/.gitkeep` para que la carpeta exista en el repo.

## Ver tambien

- [../README.md](../README.md) sección "Detección HAZMAT" — arquitectura del nodo `object_detector`.
- [../COMO_EJECUTAR.md](../COMO_EJECUTAR.md) sección "Probar la detección HAZMAT" — como correrlo.
