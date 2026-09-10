# Validación de la integración 6R

Fecha: 2026-09-10. Repositorio: `Pedros-Rescue`.
Rama: `feat/cinematica-6r-unificada`, creada desde `todo-organizado` en
`59fbf0dc518a98cfea8772b87242ec276084d9cf`.
Rama de trabajo encontrada: `test/cinematica`, commit `5c07b57`.
El árbol inicial estaba limpio. Se fusionó su contenido con la base antes de
integrar el ZIP. No hay merge pendiente, commit nuevo ni publicación remota.

## Procedencia y conservación

- ZIP: `ENTREGA_CODIGO_6R.zip`, leído desde Descargas; no se modificó el ZIP.
- Instrucciones: [INSTRUCCIONES_CODEX_6R.md](INSTRUCCIONES_CODEX_6R.md), copia íntegra.
- [MANIFIESTO_ENTREGA.json](MANIFIESTO_ENTREGA.json): SHA-256 originales, comprobados
  para los 16 archivos de integración y sus diez bases antes de instalarlos.
- [merge_report.json](merge_report.json) registra la instalación/fusión inicial,
  antes de las adaptaciones descritas a continuación.
- `kinematics.py`, `motion.py`, `cartesian_controller.py` y `simulator.py` coinciden
  byte a byte con los SHA-256 suministrados. No hubo correcciones matemáticas.
- Los **12 archivos** de `simulador_web_original` son idénticos al ZIP, incluidos
  HTML, CSS, JavaScript, README y pruebas. La vista de estación es una extracción
  reproducible del renderer, no una modificación de esa demostración.
- Los cinco valores geométricos locales de `test/cinematica` se mantienen en
  `arm.yaml`: h=.12, L1=.36, L2=.36, L3=0, herramienta=0, en metros.
  La diferencia con .14/.30/.30/.10/.10 de la entrega está explicada en
  [ARM_6R.md](../../ARM_6R.md). No se eligieron medidas a partir de un dibujo.
- Los cambios de visión/dashboard/SLAM presentes en `todo-organizado` permanecen.
  La implementación cinemática anterior se conserva en la rama `test/cinematica`,
  y se sustituye en el código activo conforme al Markdown.
- En `rescue_robot_core` solo se añadieron cuatro líneas por nodo de driver:
  guardar el último mensaje y publicar status a 2 Hz. No se modificó ningún
  contador, lectura/escritura de registros, calibración ni bucle de control.

## Adaptaciones realizadas

1. **Canvas → Tk:** recursos instalables, extracción de funciones originales,
   serialización de FK Python, worker Chromium, PNG en hilo Tk y gestos de vista.
   La escena integra tool/camera/origins/axes y preview. No carga motores JS.
   Se omiten capturas cuando no hay cambios, incluidos periodos de GUI oculta.
2. **Admisión de movimiento:** `/arm/manual_joint_cmd` centraliza las órdenes
   articulares de GUI. La planificación y la ejecución excluyen comandos nuevos
   bajo la misma reserva; el feedback sigue llegando en callbacks independientes.
3. **Tiempos ROS:** `CartesianWaypoint.duration` se interpreta como segundos
   totales por segmento; conserva diferentes tiempos entre waypoints. Se adapta
   a las funciones entregadas y al rango [0.2,120] s del planificador.
4. **Supervisión:** heartbeat de buses a 2 Hz y límites de antigüedad originales
   (status <2 s, seis articulaciones <1 s), incluso con velocidad cero. Se espera
   feedback completo antes de publicar FK medida. La vista marca feedback obsoleto.
5. **Cancelación/GUI:** cancelar también desde teleop, solicitar cancelación antes
   de desconectar/emergencia/calibración, liberar peticiones al terminar o tras
   timeout de cliente, sembrar orientación estabilizada y distinguir una trayectoria
   detenida de una completada. La cancelación no interrumpe un cálculo matemático
   a mitad de función; se comprueba antes de admitir/publicar su resultado.

## Comprobaciones ejecutadas aquí

Entorno: Windows, Python 3.11, NumPy, PyYAML, Tk, Node.js y Playwright/Chromium.
Dependencias de comprobación instaladas en un venv de trabajo fuera del repo.
No se instalaron ROS ni dependencias de drivers en esa comprobación.

| Comprobación | Resultado |
| --- | --- |
| `python -m unittest discover -s src/rescue_command_station/test -v` | **35 tests OK**: 15 de entrega/ejecutor, 10 de integración y 10 de auditoría |
| Web `node verify.cjs` | **OK**: 1200 FK independientes, 1200 poses FK/IK, 100 IK de posición, singularidades/límites |
| Web `node verify-motion.cjs` | **OK**: 20 trayectorias aceptadas, marcos/J6, interpolación, rechazos y CSV |
| `tools/verify_arm_gui.py` | **OK**: construcción de GUI y pestañas con transporte falso, ventana oculta |
| `tools/verify_arm_canvas.py --tk` | **OK**: Chromium + widget Tk aislado, sin ROS |
| Inspección de capturas Canvas | **Realizada**: configuración local y dimensiones de entrega en memoria, cámara/barra y preview de J6 |
| `setup.py build_py --build-lib <carpeta de trabajo>` | **OK**: incluye `arm/web_assets/index.html` y `canvas.js` |
| `compileall` de estación, launch y nodos core | **OK** |
| `git diff --check` | **OK**; Git solo advierte normalización futura LF/CRLF |
| Integridad SHA-256 de motores y comparación del web con ZIP | **OK** |
| Comparación de cinco medidas contra la rama local original | **OK** |

Los diez casos nuevos prueban: duración total independiente por segmento;
rechazo de comandos manuales/segundas solicitudes durante preflight y cancelación;
orden/conversión y validación de órdenes manuales; pérdida de feedback/status
en pausa; cancelación de publicación y desconexión; serialización del TCP activo
y flange; recepción completa de seis articulaciones; orientación inicial de
estabilización; liberación de petición fallida; timeout que solicita cancelación
una sola vez. El transporte de esos tests es falso y no importa ROS.

La prueba Tk usa una ventana oculta para verificar recepción de PNG y cierre del
worker. **No equivale a una revisión de la GUI ROS completa en pantalla.**

## Lo que queda por ejecutar en ROS

1. Compilar el conjunto con ROS 2/`colcon`, siguiendo [ARM_6R.md](../../ARM_6R.md).
2. Lanzar primero `ros2 launch rescue_command_station arm_station.launch.py sim:=true`.
   Sin drivers reales activos ni conexión a hardware para esta fase.
3. Inspeccionar `ros2 topic info /arm/joint_states -v`: ambos buses deben aportar
   nombres que, en conjunto, cubran las seis articulaciones; no se exige que cada
   mensaje individual tenga seis nombres. Medir ambos `/.../status` a ~2 Hz.
4. Confirmar que estados y heartbeat se reciben durante preflight largo, y que
   manual/cartesiano no compiten. Verificar la semántica total de `duration` en la
   GUI y servicios, también con más de dos waypoints.
5. Probar cancelación, pausa, fallo de plan, desconexión, emergencia, timeout y
   pérdida de solo un bus; observar que cesa el envío de nuevos puntos y que
   la GUI no anuncia falsamente «Completada». Cancelar no detiene el último
   objetivo que el driver ya recibió.
6. Revisar joystick, estabilización desde arranque, ambos TCP, límites, calibración,
   jog/rescate, scroll, cámara frontal/QR, dashboard y cierre con la pantalla real.
7. Medir latencia y consumo de Chromium/PNG en el PC de destino y comprobar sus
   dependencias Linux. La prueba de empaquetado aquí no cubre instalación ROS.

La auditoría posterior añadió exclusión de mantenimiento desde la GUI, retiró
los canales heredados y corrigió convenciones/configuración/arranque. El detalle,
las regresiones y las limitaciones todavía abiertas están en
[AUDITORIA_BRAZO.md](AUDITORIA_BRAZO.md). Los clientes externos de drivers no quedan
bajo la exclusión de la estación; los servicios reales no se reescribieron.

No se implementaron posición adicional persistente, IK Python solo de posición,
colisiones o límites dinámicos. No se midió ni accionó el brazo. Estas pruebas
matemáticas y de transporte falso **no certifican funcionamiento físico**.

## Revalidación tras auditoría

Repetir la construcción de `rescue_interfaces` por la retirada de `ComputeIK`
y los campos nuevos de `CartesianState`; no mezclar instalaciones antiguas.
Para evitar que un módulo generado retirado quede en `install`, validar primero
en un overlay limpio (`--build-base build-arm-audit --install-base install-arm-audit`),
sin borrar los artefactos locales anteriores. Sourcear únicamente el overlay
elegido sobre ROS base. Desplegar el heartbeat real en la Pi y el mismo YAML de
servos en PC/Pi; el argumento de ruta se resuelve localmente en cada máquina.

Comandos offline adicionales, con dependencias de `requirements_pc.txt`:

```bash
PYTHONPATH=src/rescue_command_station python src/rescue_command_station/tools/verify_arm_gui.py
PYTHONPATH=src/rescue_command_station python src/rescue_command_station/tools/verify_arm_canvas.py --tk
```

`MANIFIESTO_ENTREGA.json` y `merge_report.json` son evidencia del ZIP/fusión inicial,
no un inventario de hashes del árbol adaptado final. No se modifican para ocultar
las diferencias de integración y auditoría.
