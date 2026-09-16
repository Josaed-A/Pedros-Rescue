# Contexto del proyecto Pedro’s Rescue

Plan vigente para implementar limpieza y separación PC/Pi:
[10_PLAN_LIMPIEZA_Y_ARQUITECTURA.md](10_PLAN_LIMPIEZA_Y_ARQUITECTURA.md).
Decisión del usuario: órdenes y feedback por apodo lógico (`Hombro`); solo Pi
conoce bus/ID/calibración. Incluye matrices de migración/borrado, ocho fases
F0–F7 y pruebas de independencia. Solo planificación.

## Revisión vigente del perfil básico — 2026-09-15

Para movimiento, brazo/patas y transmisión de cámaras, leer primero
[09_PERFIL_BASICO_Y_EXCLUSIONES.md](09_PERFIL_BASICO_Y_EXCLUSIONES.md).
Incluye los 15 launches, seis scripts, todos los grupos de percepción/YOLO/Hazmat,
SLAM 2D, nubes 3D, exportadores y herramientas, clasificados por necesidad y
evidencia. Conserva H01–H29/D01–D10 y añade B01–B05. También rectifica las
inferencias anteriores sobre saturación, tasa de `/cmd_vel` y entorno ROS.
Es una revisión/documentación; la operación física integral sigue sin validarse.

El resto de esta introducción conserva el corte histórico del 12 de septiembre.

Esta carpeta es el contexto principal del proyecto y el punto de reanudación para cualquier IA o colaborador. Elaborado por Codex a partir del código, configuración y comprobaciones del checkout local el 2026-09-12. Incluye arquitectura, inventario, auditoría, trabajo realizado y pendientes.

Alcance actual: mantener contexto verificable y reportar estructura, arquitectura, residuos, contratos y fuentes de verdad. La auditoría no autorizó correcciones de implementación. Posteriormente el usuario pidió eliminar el contexto anterior y consolidar esta documentación; esa reorganización ya está completada.

Base Git: `3b755cec99230810fd10f5776eb4522d82439095`.
Cambios previos preservados: `requirements_pc.txt` añade zxing-cpp; `src/rescue_command_station/config/arm.yaml` cambia L3 a 0.05 y tool_length a 0.08.

Proyecto: workspace ROS 2 con siete paquetes locales. Robot/Pi: motores, cámaras y buses de servos; PC: teleoperación, dashboard, cinemática y brazo; bringup: launch, percepción y mapas. La distribución efectiva depende del launch elegido y debe comprobarse, no deducirse de los README.

Leer `01_TRABAJO.md`, luego el informe y el inventario enlazados abajo. Los SHA256 iniciales permiten verificar que esta tarea no alteró los archivos versionados.

## Entrega y navegación

Estado final: auditoría estática general completada; 29 hallazgos, sin arreglos. Se conservaron los cambios locales previos. El contexto anterior fue eliminado por petición del usuario y esta carpeta reúne la documentación vigente. No asumir validación de hardware por tener esta auditoría.

1. [Trabajo, objetivo y pendientes](01_TRABAJO.md).
2. [Hallazgos priorizados y recomendaciones no aplicadas](02_HALLAZGOS.md).
3. [Mapa de carpetas y fuentes de verdad](03_MAPA_Y_FUENTES_DE_VERDAD.md).
4. [Inventario de los 167 archivos versionados](04_INVENTARIO_ARCHIVOS.md).
5. [Contratos ROS extraídos](05_CONTRATOS_ROS.md).
6. [Comprobaciones, reproducción y límites](06_COMPROBACIONES_Y_LIMITES.md).
7. [Índice de launch](07_LAUNCH.md).
8. [Dependencias cruzadas entre paquetes Pi/PC/compartido](08_DEPENDENCIAS_CRUZADAS.md).

Evidencias auxiliares: resultados .txt, hashes iniciales .json y reproducir_hallazgos.py. No forman parte del runtime del robot.

Conclusión de arquitectura: la división de paquetes base es aprovechable. Los principales problemas están en orquestación inconsistente, contratos de datos y control no uniformes, estado de misión fragmentado y recetas de despliegue múltiples. El modelo Python del brazo sí comparte configuración y planificación; el Canvas integrado no introduce un segundo solver activo. Evitar refactorizaciones generales basadas solo en nombres de carpetas.

## Mantener el contexto actualizado

Al comenzar una tarea, leer 01_TRABAJO.md y comprobar el estado real del repositorio. Registrar allí el objetivo vigente, avances y pendientes durante el trabajo. Actualizar el mapa y los contratos cuando cambien; conservar fecha y alcance de las evidencias de auditoría para no presentarlas como pruebas de versiones posteriores. Mantener esta carpeta como único contexto general, sin crear otra copia paralela.

## Separación PC/robot — revisión ampliada

Conclusión vigente en [08_DEPENDENCIAS_CRUZADAS.md](08_DEPENDENCIAS_CRUZADAS.md): PC y robot deberían instalarse y probarse separadamente, compartiendo contratos ROS y únicamente los recursos necesarios. Hoy station exige core por configuración y simulación; bringup mezcla perfiles. No se encontraron imports Python de drivers core en station ni de GUI station en core. Se precisó la diferencia entre instalar un paquete y ejecutar hardware. D01–D10 están revisados; la afirmación anterior de Dockerfile PC huérfano fue retirada. El desacople es una propuesta documentada, todavía no implementada.
