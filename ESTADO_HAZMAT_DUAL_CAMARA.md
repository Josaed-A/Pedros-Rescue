# Estado actual — Detección HAZMAT dual-cámara

Snapshot de esta sesión de trabajo sobre la rama `Test`. Resume qué se
implementó, qué se verificó funcionando, y un bug encontrado que **todavía
no está corregido** (diagnóstico completo, sin fix aplicado a propósito).

## Qué se implementó

1. **Prueba local de dos cámaras** ([test_local_cameras.launch.py](src/rescue_bringup/launch/test_local_cameras.launch.py)):
   sustituye la Astra + Logitech del robot real por las dos cámaras de un PC
   de escritorio (GENERAL WEBCAM + cámara local integrada), cada una con su
   propio `logitech_pub` (su propio `cv2.VideoCapture`, sin compartir).
   **No toca** `pi_sensors.launch.py` / `pedro_pi.launch.py` /
   `logitech_vision.launch.py` / `command_station.launch.py` — el robot real
   sigue exactamente igual.

2. **Worker HAZMAT compartido** ([hazmat_worker.py](src/rescue_bringup/rescue_bringup/hazmat_worker.py) +
   [hazmat_common.py](src/rescue_bringup/rescue_bringup/hazmat_common.py)):
   un solo proceso carga el modelo YOLO hazmat **una sola vez** y sirve a
   ambas cámaras vía `/hazmat/submit/<camera_id>` → `/hazmat/result/<camera_id>`.
   Scheduler de prioridad: round-robin normal, boost a la cámara que confirma
   señal, piso de vigilancia para la otra (nunca se detiene), reversión a
   normal tras un timeout configurable.

3. **object_detector.py**: nuevo parámetro `hazmat_mode` (`local` default =
   comportamiento de siempre sin cambios; `worker` = no carga modelo propio,
   reenvía frames al worker y consume sus resultados). AprilTag, YOLO-objetos,
   dibujo, alertas, CSV y RViz quedaron intactos y se reutilizan sin tocar.

4. **Capturas automáticas de alertas hazmat**: una detección solo se confirma
   (y se guarda) tras sostenerse varios frames seguidos — evita ruido de un
   parpadeo del modelo. Se guarda en `hazmat/alertas_detectadas/<camera_id>/`
   (frame anotado `.jpg` + descripción `.json`: clase, confianza, bbox, cámara).

5. **Reorganización**: todo lo hazmat (modelos, entrenamiento, prueba OpenCV
   standalone, capturas de alertas) vive bajo [hazmat/](hazmat/README.md).
   `training/` se movió a `hazmat/training/`.

## Qué se verificó funcionando (en vivo, con las 2 cámaras reales del PC)

- Ambas cámaras capturan simultáneamente a su FPS normal (~15 Hz), sin bajar
  nunca, sin importar el estado del scheduler de prioridad.
- El modelo hazmat se carga **una sola vez** (log del worker), ninguna
  instancia de `object_detector` carga su propia copia en modo `worker`.
- AprilTag y YOLO-objetos siguen funcionando igual en cada cámara.
- Inyección directa a `/hazmat/result/front` y `/hazmat/result/astra`
  confirmó que cada resultado llega, se dibuja y se guarda en la carpeta
  correcta (`hazmat/alertas_detectadas/front/` y `.../astra/` respectivamente)
  — sin mezcla entre cámaras en esa ruta.
- Scheduler de prioridad probado de forma aislada: boost tras confirmación,
  piso de vigilancia real para la cámara demovida, reversión tras timeout.

## Bug encontrado — NO CORREGIDO todavía

**Síntoma:** durante la prueba con ambas cámaras corriendo detección a la
vez, los popups de detección en el dashboard pueden mostrar el frame de la
cámara equivocada (p. ej. una señal detectada por `astra` se presenta junto
a una imagen de `front`), dando la sensación de detecciones intercaladas o
"de la cámara equivocada".

**Causa raíz:** `dashboard_node.py::detections_callback` recibe detecciones
de **ambas** cámaras por el mismo topic `/object_detections` (ninguna
instancia lo remapea), pero arma el snapshot del popup **siempre** con
`latest_front_annotated_frame`/`latest_front_frame`, sin mirar de qué cámara
vino la detección — porque el JSON publicado en `/object_detections`
(`object_detector.py::_process_detection`) tampoco lleva un campo
`camera_id`. Es un bug preexistente que estaba dormido porque antes solo
`front` producía detecciones; al darle a `astra` su propio `object_detector`
funcional, quedó expuesto.

**Verificado que NO está afectado:** el pipeline hazmat completo
(submit → worker → scheduler → inferencia → result) y el guardado en disco
(`hazmat/alertas_detectadas/<camera_id>/`) — ahí el `camera_id` se mantiene
correcto en todo momento, confirmado con pruebas de inyección en vivo.

**Corrección propuesta (pendiente, no implementada):**
1. `object_detector.py::_process_detection` — añadir `'camera_id': self._camera_id`
   al JSON de `/object_detections`.
2. `dashboard_node.py::detections_callback` — elegir el snapshot (`front` vs
   `astra`) según ese campo, con fallback a `front` si falta (compatibilidad
   con el robot real, donde nunca se setea y solo hay una cámara con detección).

Diagnóstico completo (con trazado línea por línea, hipótesis descartadas y
prueba reproducible propuesta) en el historial de esta conversación — no se
duplica aquí para no desactualizarse si el fix cambia de forma.

## Qué falta

- Aplicar la corrección del bug de arriba (pendiente de decisión/aprobación).
- Confirmar si el "piso de vigilancia" / tiempos del scheduler (`priority_confirm_frames`,
  `priority_release_sec`, `demoted_period_ticks`) necesitan ajuste con hardware
  real (Astra + Logitech) en vez de las cámaras de prueba del PC.
- Decidir si esta arquitectura (worker compartido) se lleva también a
  `pi_sensors.launch.py` para el robot real, o se queda solo como herramienta
  de prueba local.
