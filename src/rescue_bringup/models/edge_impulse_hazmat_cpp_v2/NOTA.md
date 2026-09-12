# Edge Impulse HAZMAT v2 — metadata de referencia

Proyecto Edge Impulse: https://studio.edgeimpulse.com/studio/1110262 (impulse "HAZMAT")

Export original: `hazmat-cpp-mcu-v2-impulse-#1.zip` (Downloads/Archives, descargado 2026-09-10).

## Por qué solo están los metadatos aquí

El export completo es una librería **C++ para microcontroladores** (SDK con drivers
Ethos-U NPU, tablas CMSIS-DSP para Cortex-M, ARC MLI), pensada para correr el modelo
en bare-metal/MCU — no para este robot, que corre el pipeline de detección en
**Python vía ultralytics** (`src/rescue_bringup/rescue_bringup/object_detector.py`,
carga `.pt` con `YOLO(hazmat_model_path)`).

Se descartó el SDK embebido y el modelo compilado (`tflite_learn_*_compiled.cpp`,
122MB — además excede el límite de 100MB por archivo de GitHub). Solo se conservan
`model-parameters/` (metadata) y el `README.txt` original como referencia.

## Datos del modelo (de `model-parameters/model_metadata.h`)

- Input: 96×96
- Object detection, last layer YOLO-Pro
- 15 clases: ERIrotation, Placards, corrosive, dangerous-when-wet, explosive,
  flammable, flammable-solid, infectious-substance, license plate,
  non-flammable-gas, organic-peroxide, oxidizer, poison, radioactive,
  spontaneously-combustible

## Para usarlo de verdad en este repo

Haría falta volver a exportar desde Edge Impulse Studio en formato **TFLite** u
**ONNX** (deploy → "TensorFlow Lite" o similar) en vez de "C++ library", y luego
adaptar `object_detector.py` para cargarlo (hoy solo sabe cargar `.pt` vía
ultralytics). Esa integración no se hizo — pendiente si se decide seguir este camino.
