# Mapa del proyecto y fuentes de verdad

Corte: 2026-09-12. Fuente: archivos del checkout, manifiestos, imports, launch y consumidores; no la autoridad de un README. El inventario individual completo está en [04_INVENTARIO_ARCHIVOS.md](04_INVENTARIO_ARCHIVOS.md).

## Organización por carpeta

| Ruta | Qué contiene / para qué sirve | Clasificación y observación |
|---|---|---|
| `docs/README.md` | Enlace al contexto principal y guía de navegación | Entrada de documentación. |
| `.git/` | Historial, índice, objetos (~95 MiB) | Metadatos necesarios; no borrar ni tratar como código activo. |
| `.vscode/` | Configuración local de editor | Ignorada; preferencias locales, no requisito de ejecución. |
| `docs/contexto_proyecto/` | Contexto general de Codex: auditoría, estado, mapa, inventarios y evidencias | Punto de entrada vigente del proyecto. Los scripts anexos son reproducciones de auditoría, no parte del producto. |
| `build/` | Salidas colcon de siete paquetes; CMake, generación rosidl, enlaces Python | Generado (~20 MiB), ignorado. No editar. Contiene referencias Python 3.12. |
| `install/` | Overlay de siete paquetes, scripts de entorno, recursos y extensiones | Generado (~5.6 MiB), ignorado; necesario para ejecutar un build, no segunda implementación. Enlaces absolutos al checkout. |
| `log/` | Logs de tres builds y enlaces latest | Generado (~1.7 MiB), útil para diagnóstico. No prueba que este entorno pueda reproducir el build. |
| `scripts/` | Red PC/Pi, contenedores, permisos USB, autostart systemd | Operación activa; rutas, red, credencial y sincronización parcial requieren revisión. |
| `training/` | Entrena hazmat, exporta ONNX y prueba cámara interactiva | Herramientas auxiliares; test_hazmat_camera no es una prueba unitaria y abre cámara. Dataset ausente/ignorado. |
| `src/Slam/` | Subdirectorios hector_slam_ros2, ldrobot-lidar-ros2, Slam_3d/OrbbecSDK_ROS2 y ros2_astra_camera | Vacíos y sin archivos versionados; residuo/placeholder, no proveedor de dependencias. |
| `src/dependencias/` | Dos paquetes Python propios | No es una descarga íntegra de terceros; reemplaza paquetes conocidos por versiones mínimas. |
| `src/dependencias/joy/joy/` | Lectura Linux joystick, publicación Joy, autorepeat | Activo; bug de desconexión H04. resource/setup/config/package registran paquete. |
| `src/dependencias/cv_bridge/cv_bridge/` | Conversor numpy↔Image parcial | Activo; colisión de nombre y falta 32FC1 H08. resource/setup/config/package registran paquete. |
| `src/rescue_interfaces/msg/` | ArmStatus, CartesianState, CartesianWaypoint | Contratos compartidos. CMakeLists genera código; package.xml declara std_msgs y rosidl. |
| `src/rescue_interfaces/srv/` | CartesianGoto, CartesianTrajectory, ComputeIKPose, RegisterServo, ServoCommand, ServoStatus | Servicios propios; semántica asíncrona de ejecución y porcentajes debe preservarse o migrarse expresamente. |
| `src/rescue_robot_core/config/` | servos.yaml: buses, puertos, IDs, relaciones, inversión y patas | Configuración física; no es geometría del brazo. Valores provisionales señalados en el propio YAML. |
| `src/rescue_robot_core/launch/` | robot_core: motores+cámaras propias; servos: AX+EX | Dos ámbitos diferentes; robot_core por sí solo no incluye servos/lidar. |
| `src/rescue_robot_core/rescue_robot_core/nodes/` | Motor BTS7960, bus AX brazo+patas, EX hombro, driver simulado | Drivers activos; AX/EX comparten muchas ramas pero tienen diferencias físicas. Simulado ofrece protocolo de brazo, no valida comportamiento físico de patas. |
| `…/rescue_robot_core/camera_drivers/` | Logitech robusto, Astra V4L2, serialización imagen y proyección PointCloud2 | Drivers propios; Astra no implementa OpenNI pese a comentario de un launch. |
| `…/rescue_robot_core/config/` | motor_config.py: GPIO, ganancias, PWM, rampas y timeout | Defaults de motores; algunos se repiten/overridean en launch. Distinto del config/servos.yaml exterior. |
| `…/rescue_robot_core/drivers/` | bts7960.py: escritura de PWM | Capa pequeña activa, consumida por motor_driver_node. |
| `…/rescue_robot_core/motion/` | differential_drive.py y s_curve.py | Mezcla normalizada y perfiles de salida, no modelo cinemático con odometría. |
| `…/rescue_robot_core/servos/` | params.py y wheel_encoder.py | Lectura configuración y acumulación de posición con reductora; usada por ambos drivers/simulación. |
| `src/rescue_command_station/config/` | arm.yaml | Geometría, marco TCP, orden, signos lógicos y límites. Incluye cambios locales del usuario preservados. |
| `src/rescue_command_station/launch/` | command_station y arm_station | Dashboard/teleop y brazo real o simulado. Dashboard también arranca arm_station por subprocess. |
| `…/rescue_command_station/control/` | config, gearbox y tank_drive | Mapeo PS4, cinco marchas y mezcla de conducción. |
| `…/rescue_command_station/input/` | ps4_controller.py | Interpreta Joy; separado del nodo que lee el dispositivo. No es duplicación del driver joy. |
| `…/rescue_command_station/vision/` | QR zxingcpp, conversión de imagen y utilidades Tk | Consumo en UI; conversor de crudas tiene limitación de padding. |
| `…/rescue_command_station/nodes/` | ps4_teleop, dashboard, visor RGBD | UI/teleoperación; dashboard también administra patas, archivos, ventanas y procesos. |
| `…/rescue_command_station/arm/` | configuration, kinematics, motion, utilidades SLERP/tolerancia, nodos de cinemática/cartesiano/GUI, simulator y web_view | Núcleo matemático reutilizado y adaptadores. GUI mezcla muchas responsabilidades; simulador Python es offline. |
| `…/arm/web_assets/` | index.html y canvas.js | Vista integrada activa: recibe FK ya calculada en Python, sin resolver IK en JS. canvas.js es generado y versionado. |
| `src/rescue_command_station/simulador_web_original/dist/` | HTML/CSS/JS del simulador original (modelo, IK y movimiento propios) | Distribución estática íntegra; el nombre dist no prueba que sea código compilado. app.js y kinematics.js alimentan el generador del Canvas. No borrar como basura. |
| `src/rescue_command_station/simulador_web_original/` | README y verificadores matemáticos .cjs | Referencia/demostración offline; no se conecta a ROS. No se ejecutó por ausencia de Node en PATH. |
| `src/rescue_command_station/tools/` | extract_arm_canvas, verify_arm_canvas, verify_arm_gui | Generador activo y verificadores; extracción exacta comprobada interceptando escrituras. |
| `src/rescue_command_station/test/` | Cuatro módulos unittest compatibles con pytest | Pruebas de brazo/adapter, algunas con AST y transporte falso. Falta cobertura de otras áreas. |
| `src/rescue_command_station/docs/integracion_6r/` | Auditoría, instrucciones, validación, manifiesto JSON y reporte de merge | Trazabilidad histórica, no garantía del código actual. |
| `src/rescue_bringup/launch/` | 11 launch: PC/Pi, sensores, visión, cámaras, lidar, SLAM, descripción, guardado y full_bringup | Orquestación con variantes incompatibles y argumentos sin consumo; no todas son recetas equivalentes. |
| `src/rescue_bringup/config/` | ld19_params.yaml, slam_toolbox_params.yaml, slam_rviz.rviz | Defaults y visualización; launch puede sobreescribir valores. |
| `src/rescue_bringup/models/` | hazmat_yolo.pt, hazmat_yolo.onnx, mission_objects_yolo.pt | Binarios activos versionados (~55 MiB), no caches. Registrar linaje y paridad. |
| `src/rescue_bringup/rescue_bringup/` | object_detector, scan_merger, pointcloud_accumulator, geotiff_writer y logitech_pub | Lógica funcional además de integración. logitech_pub duplica capacidad del driver Logitech de core con menos gestión de reconexión. |
| `src/rescue_robot_description/urdf/` | rescue_robot.urdf.xacro | Descripción de base/ruedas/lidar/cámara fija. No incluye brazo/patas. |
| `resource/` de paquetes Python | Marcador ament_index | Archivos vacíos necesarios para descubrimiento, no basura. |
| `__init__.py`, setup.py/setup.cfg y package.xml | Importabilidad, instalación y metadatos ROS | Incluso vacíos pueden ser estructurales. No limpiar por tamaño cero. |
| `__pycache__/` | Bytecode de ejecuciones previas dentro de src | Cache ignorada y regenerable. Las pruebas de esta auditoría deshabilitan bytecode. |

En raíz: README.md y COMO_EJECUTAR.md son guías operativas a contrastar; Dockerfile/Dockerfile.pi definen entornos base; requirements_pc/raspberry.txt listan pip; system_requirements_pc/raspberry.txt listan apt; 99-pedros-rescue.rules define aliases USB; .gitignore/.gitattributes gobiernan seguimiento y finales de línea. No se encontraron CI, lockfile general, .repos ni .gitmodules versionados.

## Flujos que realmente expresa el código

- Conducción: joy local → `/joy` → ps4_teleop → `/cmd_vel` normalizado → motor_driver → perfil S/PWM. Feedback `/real_speed_abs` es salida calculada, no sensor de velocidad.
- Patas: `/joy` → dashboard → `/legs/cmd` (JointState) → mismo bus AX del brazo; retorno `/legs/state`. El driver tiene timeout de comandos, condicionado por poder entrar a su rama de control.
- Brazo: GUI → IK de preview o servicio cartesiano; órdenes manuales → `/arm/manual_joint_cmd` → cartesian_node → `/ax12a/joint_cmd` + `/ex106/joint_cmd`; drivers → `/arm/joint_states` por remapping. Cinemática sirve `/compute_ik_pose` y publica TCP/tool/camera. La GUI conserva servicios directos de mantenimiento.
- Cámara: caminos alternativos core (`/robot/camera/*` comprimidos) y SDK (`/camera/*` crudos). No son intercambiables solo cambiando un nombre de launch. Logitech tiene dos publicadores posibles que compiten si se arrancan juntos.
- SLAM: LiDAR + depthimage_to_laserscan → scan_merger → `/scan_merged` → slam_toolbox → `/map` y map→odom; odom→base_footprint es estático. Nube se acumula en map por separado.
- Datos de misión: detector → JSON `/object_detections` → mapa/dashboard; manuales salen del dashboard por el mismo topic, pero no entran al CSV automático. Guardado usa tres servicios independientes, sin transacción ni identificador común de misión.

## Matriz de fuentes de verdad

| Dato / capacidad | Fuentes observadas | Veredicto |
|---|---|---|
| Geometría brazo ROS | arm.yaml → configuration.py → kinematics.py | Centralización correcta en el camino Python. Overrides por nodo pueden divergir; no hay fingerprint entre PC/Pi. |
| Simulador original | dist/kinematics.js + configuración web | Segundo modelo real, pero aislado de ROS. Mantener como referencia identificada, no editar suponiendo que cambia el robot. |
| Dibujo integrado | dist/app.js + extract_arm_canvas → web_assets/canvas.js | Fuente/derivado explícitos. Comparación realizada: COINCIDEN. Editar derivado directamente se perdería al regenerar. |
| Calibración física | servos.yaml, parámetros driver, calibración/registro runtime | Snapshot inicial compartido, estado runtime mutable; no persistencia/propagación de cambios garantizada. |
| Montaje físico robot | URDF, arm.yaml, intrínsecos en nodos/launch, ROBOT_HALF_LENGTH | Descripción parcial y constantes repetidas; falta unir marcos y mediciones. |
| Red PC/Pi | ros_net_*.sh, pedro_*.launch.py, XML en contenedores | Implementaciones paralelas con defaults y autodetección distintos. |
| Componentes que arrancan | README/COMO, scripts, pedro_pi, pi_sensors, full_bringup | No existe receta coherente única; duplicación de servos y flags ignorados confirmados. |
| Dependencias | package.xml, setup.py, requirements, system_requirements, Docker, pip runtime | Fuentes parciales y en desacuerdo. Separar rol de metadatos y receta sin duplicar listas independientes. |
| cv_bridge / joy | Sistema ROS + overlays locales homónimos | Misma identidad de paquete con implementación distinta; orden de overlay decide. |
| Captura Logitech | core/logitech_camera_node y bringup/logitech_pub | Duplicación funcional real, usada por launch distintos. Requiere elegir proveedor por perfil. |
| Estado de misión | mapping_active UI + start_time de cada nodo | Varias autoridades sin sincronización: fallo funcional. |
| Detecciones/IDs | Lista detector, manual_detections dashboard, objetos mapa | Tres registros con políticas distintas, sin ID canónico ni esquema ROS específico. |
| Exportaciones | Parámetros de cada nodo + constantes dashboard/scripts | Rutas/equipo/tiempos/marcos divergen. |
| Modelos PT/ONNX | Binarios versionados + training + metadata implícita de pesos | No se comprobó linaje ni equivalencia; no declarar uno equivalente al otro. |
| Código ejecutado | src + overlay install + copias Pi sincronizadas parcialmente | install es derivado, no autoridad editable; Pi puede ejecutar revisión distinta. |
| Contexto para IA | docs/contexto_proyecto, elaborado por Codex | Único contexto general vigente. Los documentos de integración dentro de src son antecedentes específicos; comprobar evidencia al cambiar checkout. |

## Qué se puede llamar basura y qué no

Candidatos a limpieza posterior: caches __pycache__, logs antiguos tras conservar evidencia útil, placeholders vacíos de src/Slam. build/install son regenerables pero borrarlos elimina el entorno construido; no hacerlo hasta disponer de receta de reconstrucción. .vscode puede ser preferencia útil. Los modelos, recursos vacíos ament, __init__.py, dist original, generador y docs históricas tienen consumidores o valor de trazabilidad: no hay base para borrarlos indiscriminadamente. Durante la auditoría no se eliminó nada. En la consolidación posterior se eliminaron únicamente los tres MD del contexto general anterior, por petición del usuario.

## Ampliación: independencia entre máquinas

Ver [08_DEPENDENCIAS_CRUZADAS.md](08_DEPENDENCIAS_CRUZADAS.md) y su evidencia AST/manifiestos. La separación física PC/Pi no coincide todavía con las dependencias de instalación: station necesita recursos/simulación de core y bringup exige ambos lados. El grafo declarado es acíclico; la búsqueda RViz de station dentro de bringup es una dependencia adicional de recursos. Contratos compartidos como rescue_interfaces son una frontera válida, no duplicación ni dependencia de drivers. Los paquetes/propuestas de separación descritos en 08 no existen aún y no deben confundirse con este mapa del código actual.
