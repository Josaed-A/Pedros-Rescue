# Auditoría del perfil básico: movimiento, brazo y cámaras

Plan de ejecución derivado de esta auditoría y de la decisión del usuario sobre
apodos: [10_PLAN_LIMPIEZA_Y_ARQUITECTURA.md](10_PLAN_LIMPIEZA_Y_ARQUITECTURA.md).
La auditoría clasifica fallos; el plan fija secuencia y condiciones de retirada.

Fecha: 2026-09-15. Base: `ea41b28284ac066ae95b3d2642533453d348daae`.
Petición: identificar lo que impide operar lo básico y enumerar lo que queda
fuera, incluyendo detección, mapas, launches, scripts y residuos. Esta entrega
es documentación; no corrige drivers, no elimina componentes ni prueba movimiento.

## 1. Conclusión y alcance de la evidencia

El perfil básico necesita conducción, control de brazo/patas y vídeo RGB. No
necesita YOLO, Hazmat, AprilTags, lidar, SLAM, acumulación 3D ni exportación de
misiones. Hoy no existe un launch Pi que reúna exactamente ese perfil sin
duplicaciones, componentes adicionales o limitaciones de cámara.

Clasificación empleada:

- **Fallo estático/reproducido**: hay una ruta de código o reproducción que lo demuestra.
- **Observado en sesión**: ocurrió en intentos anteriores de esta conversación;
  no demuestra que el dispositivo siga en ese estado.
- **Condicional/no validado**: depende de instalación, cámara, cableado o configuración.
- **Fuera del básico**: puede ser útil y funcionar, pero no hace falta para este objetivo.
- **Soporte/alternativa**: no participa necesariamente en el runtime; no es basura.

No se puede garantizar una lista de todos los defectos latentes de hardware o
software. Este informe cubre los 15 launches locales, los seis scripts, los
módulos funcionales, modelos, entrenamiento, configuración y herramientas del
checkout, y enlaza todos los H01–H29 y D01–D10 anteriores.

### Comprobaciones nuevas y rectificaciones

1. En el host real, Python 3.12 importa `rclpy` y `yaml` al cargar Jazzy.
   H06 describe el entorno aislado de la aplicación con Python 3.13; no prueba
   que ROS esté roto en el PC. Para inspeccionar desde VS Code Flatpak se usó
   `flatpak-spawn --host bash -lc '…'`.
2. `pedro_pc`, `pedro_pi` y `pi_sensors` de `install` del PC coinciden byte a byte
   con sus fuentes locales. No se comprobó la igualdad de versiones en la Pi.
3. En esta revisión el PC tiene `wlan0=172.23.12.235/23`, pero sigue vivo el
   launch PID 13325 con `pc_wifi_ip:=10.230.234.1` y peer `10.230.234.137`.
   TCP/22 a esa Pi agotó el plazo de tres segundos. Es evidencia de configuración
   de sesión desactualizada, no prueba de que la Raspberry esté apagada.
4. Se reejecutó `reproducir_hallazgos.py` con Python del host: H04, H08, H09,
   H16 y H17 se reproducen; la comprobación geométrica de H07 también pasa.
   Son pruebas aisladas sin ROS activo ni dispositivos, no pruebas de conducción.
5. **Rectificación de la sesión anterior:** 40 Hz en `/cmd_vel` no demuestra dos
   teleoperadores. `ps4_teleop_node.py:82–88,145–173` publica por timer y al
   recibir Joy. Además `joy_node` publica por cambios y por repetición. Para
   detectar duplicados hay que contar procesos y endpoints publicadores.
6. Se observaron procesos duplicados en el PC durante los reinicios anteriores,
   pero esa evidencia es distinta de la tasa del tópico. `nohup` más señales
   dirigidas solo al padre dejó hijos vivos; revisar el árbol y grupos de procesos.
7. **Rectificación sobre cámaras:** hubo pérdida de SSH tras intentos de arranque,
   errores de apertura de cámara y procesos externos OpenNI2/V4L2. No se midieron
   ancho de banda, CPU, temperatura, alimentación ni logs del kernel. Saturación
   por cámaras y autostart externo son hipótesis; también falló el acceso después
   de pedir cámaras desactivadas. No atribuir una causa definitiva ni pedir
   reinicios repetidos suponiendo que ya se conoce el problema.
8. Los logs remotos anteriores mostraron `astra_rgbd_camera_node` desde el
   arranque Pi, mientras el `pi_sensors` local pide `astra_camera` y un relay
   inexistente. Esto obliga a comparar fuente e instalación remotas; no aplicar
   automáticamente la auditoría local a lo que ejecuta la Raspberry.
9. B05 reproducido extrayendo `_auto_connect_once` con AST y dos servicios falsos:
   primera llamada con disponibilidad `[True, False]`, segunda con `[True, True]`.
   Solo el primer servicio recibió `call_async`; el segundo quedó sin llamada.
   No se conectaron buses ni se habilitó torque en esta prueba.
10. Verificación de cobertura: los 15 archivos `*/launch/*.launch.py` y los
    seis `scripts/*.sh` están nombrados en este informe. `git diff --check` pasó.

## 2. Qué necesita funcionar para el básico

| Función | PC | Pi | Contrato principal | Estado |
|---|---|---|---|---|
| Orugas | `joy_node`, `ps4_teleop_node`, mezcla/cajas | `motor_driver_node`, BTS7960, perfil S | `/joy` → `/cmd_vel`; `/real_speed_abs` de retorno | Nodos iniciados antes; movimiento real y parada no validados; H04/H24 |
| Patas | dashboard, joystick derecho | `dynamixel_bus_node` AX-12A | `/legs/cmd`, `/legs/state`, calibración/torque | H05, configuración y buses pendientes |
| Brazo | `cinematica_node`, `cartesian_node`, GUI y motor 6R | AX-12A y EX-106+ | `/arm/manual_joint_cmd`, `/ax12a/joint_cmd`, `/ex106/joint_cmd`, `/arm/joint_states`, servicios cartesianos | GUI/nodos iniciados; ejecución física/calibración sin acreditar; H01/H05/H23/H25 |
| Vídeo frontal RGB | dashboard y vista de brazo | un único publicador Logitech | `/robot/camera/front/image_raw/compressed`, JPEG | Contrato compatible; dispositivo y flujo sostenido no validados |
| Vídeo RGB Astra | dashboard | backend RGB válido, publicación/remapeo comprimido | `/robot/camera/astra/color/image_raw/compressed`, JPEG | Ruta principal rota por H02/H03; alternativa propia depende de V4L2 |
| Profundidad Astra | visor RGB-D opcional | backend real de profundidad | PNG comprimido o Image + CameraInfo según consumidor | No necesaria para vídeo RGB; no funciona por defecto en backend propio, H10 |

Dependencias que se conservan: `rescue_interfaces`, configuración de servos y
brazo, OpenCV/NumPy, serial/Dynamixel/GPIO en Pi, ROS/DDS, Tk/CustomTkinter,
Playwright/Chromium y activos Canvas para la GUI del brazo. La instalación actual
de station exige recursos de core incluso en PC real (D03); no retirar core
sin resolver esa dependencia. QR es opcional funcionalmente, pero `zxingcpp`
se importa obligatoriamente en el dashboard mediante `vision/qr_detector.py`.

## 3. Bloqueos y limitaciones de movimiento y brazo

| ID/evidencia | Problema | Efecto y acción pendiente |
|---|---|---|
| H01, `pedro_pi` + `pi_sensors` | Dos inclusiones de `servos.launch.py` | Un solo propietario por bus. Confirmar expansión real de launch antes de operar |
| H04, `joy/joy_node.py:poll,publish` | Error de lectura conserva ejes/botones y renueva hora | Timeout de teleop no detecta desconexión física; reproducido |
| Sesión anterior | Ejes alrededor de -0.11 y 0.06 produjeron velocidades pequeñas | Posible deriva o mando desplazado; deadzone 0.05 no los anula; verificar centrado/calibración |
| H05, bucles AX/EX | Error de lectura hace `continue` antes de controles de parada | Caducidad de patas y control de atasco no se ejecutan en esa rama |
| H23, drivers/ejecutor | Jog directo admite valores sin validación equivalente; cancelar deja de publicar | No garantiza detener un objetivo ya recibido; definir parada efectiva y límites locales |
| H24, motor_driver | `/real_speed_abs` es salida normalizada, no encoder; Twist usa mezcla normalizada | No demostrar movimiento real con ese tópico ni conectarlo a navegación física sin adaptar |
| H24 | Watchdog refrescado antes de validación; NaN no rechazado explícitamente en toda la cadena | Validar órdenes finitas y reacción del driver |
| H25, `arm.yaml`, `servos.yaml` | Calibración, límites y algunos montajes provisionales | Validar signos, IDs, cero, relación mecánica, recorrido y feedback; L3=0.05, herramienta=0.08 |
| H22, dashboard `_preload_arm` | Dashboard ya lanza brazo oculto | No iniciar otro `arm_station` real en paralelo ni simulación con mismos nombres |
| Nuevo B05, dashboard `_auto_connect_once` | Marca intento como realizado cuando al menos un servicio está disponible, sin esperar al otro ni comprobar resultados | Si AX aparece antes que EX, EX no recibe autoconexión posterior; no equivale a ambos buses conectados |
| D01–D03/H20 | Paquetes PC/Pi comparten dependencias de instalación y recursos | Poder importar un módulo no acredita que el perfil instalado tenga todos sus recursos |
| Sesión anterior/udev | Alias AX y EX aparecieron; antes también faltó AX | Verificar `/dev/ax12a`, `/dev/ex106`, permisos e IDs en cada arranque, no inferir conexión por existencia |
| `motor_config.py`, BTS7960 | Pines BCM, polaridad y alimentación dependientes del montaje | Necesarios; sin prueba física de giro, watchdog y emergencia no declarar conducción aprobada |

## 4. Todo lo pendiente en cámaras y transmisión

- **H02:** el padre detecta USB y propone índices, driver/FPS/calidad, pero
  `pi_sensors` fija Astra SDK y Logitech dispositivo 0; no consume esos argumentos.
- **H03:** `astra_relay` no tiene módulo ni entry point local. Falta la adaptación
  de `/camera/color/image_raw` al canal comprimido que consume el dashboard.
- **H29:** habilitar Logitech mediante `pi_sensors` inicia `object_detector` aunque
  pase `launch_detector=false`: el hijo no lo consulta. No es vídeo básico puro.
- **H10:** `astra_rgbd_camera_node` usa V4L2; `depth_index=-1` abre ningún canal de
  profundidad. No implementa el fallback OpenNI anunciado en comentarios.
- No publica CameraInfo; usa intrínsecos de ejemplo. Un frame YUYV de color no
  se convierte en profundidad métrica por reinterpretar sus bytes.
- **Nuevo B01:** `logitech_pub.py:22–25` retorna del constructor si no abre la
  cámara, sin crear timer de recuperación. Si falla `read` después, descarta
  frame sin reabrir. Puede quedar vivo sin vídeo. La alternativa
  `logitech_camera_node` sí tiene reapertura y estadísticas; no son equivalentes.
- **Nuevo B02:** dashboard `:139–140,254–258` suscribe profundidad y nube solo
  para marcar presencia de la Raspberry (`mark_raspberry_seen`). No muestra
  esos datos allí, pero puede activar transmisión hacia el PC. Para RGB básico
  se puede obtener presencia desde telemetría ligera; no son callbacks vacíos.
- `camera.launch.py` fuerza `/dev/video2` en V4L2/OpenNI2 y rutas `/workspace`
  para Astra SDK. Nombre de dispositivo y librerías deben existir en esa máquina.
- El backend OpenNI2 pide `openni2_camera_node`; en la sesión remota se vio
  `openni2_camera_driver`. Comprobar `ros2 pkg executables openni2_camera` en Pi.
  En esta revisión el paquete no está instalado en PC: no demuestra su ausencia en Pi.
- No se verificó compatibilidad de cada modelo físico con Orbbec/Astra SDK;
  comentarios «siempre funciona» no son validación de cámara/driver.
- **Nuevo B03 (condicional):** detector comprimido usa BEST_EFFORT, pero el modo
  raw y CameraInfo pasan QoS por defecto (cola 5). Si el emisor raw solo ofrece
  BEST_EFFORT, revisar incompatibilidad de fiabilidad mediante endpoints reales.
  Dashboard y drivers propios sí seleccionan BEST_EFFORT explícitamente.
- **H09:** decodificador raw de estación falla con padding RGB válido; JPEG/PNG
  no pasan por esa función. **H08:** `cv_bridge` local no soporta 32FC1.
- Dos procesos abriendo el mismo dispositivo pueden competir. Se vieron cámaras
  externas OpenNI2/V4L2 durante la sesión, pero no se confirmó autostart ni su origen.
- Nodos y tópicos existentes no prueban imágenes recibidas. Se necesitan muestras,
  tasa sostenida, dimensiones/encoding válidos y visualización efectiva.

## 5. Inventario de los 15 launches

Rutas relativas a `src/`. «Excluir» significa no ejecutar en el perfil básico,
no borrar archivos. Los launches de menor nivel necesitan entorno DDS preparado.

| Launch | Papel / básico | Fallos, dependencias o límites |
|---|---|---|
| `rescue_bringup/launch/pedro_pc.launch.py` | Orquestador PC adaptable | `launch_slam:=false` excluye SLAM/exportadores/detector anidado; `launch_dashboard:=false` también quita joy/teleop, no solo ventana. IP/interfaz históricas. RViz de UI es independiente |
| `rescue_bringup/launch/pedro_pi.launch.py` | Orquestador Pi problemático | H01/H02/H29; doble bus, parámetros ignorados; no asumir que desactivar cámaras resuelve lo demás |
| `rescue_bringup/launch/pi_sensors.launch.py` | No es perfil mínimo | No lanza motores; sí TF/lidar/cámaras/servos; relay ausente, detector no desactivable por argumento recibido |
| `rescue_robot_core/launch/robot_core.launch.py` | Motores y cámaras propias | Sin servos; cámaras incondicionales; profundidad -1; índices 0/2; no configura peers CycloneDDS |
| `rescue_robot_core/launch/servos.launch.py` | Necesario una vez en Pi | Solo drivers; requiere alias/buses/configuración correcta; H05/H23 |
| `rescue_command_station/launch/command_station.launch.py` | Núcleo PC | Joy + teleop + dashboard; este precarga brazo. No configura peer/IP por sí mismo |
| `rescue_command_station/launch/arm_station.launch.py` | Brazo solo / simulación | Core exigido aun real; no coexistir con precarga de dashboard; `sim:=true` no prueba hardware |
| `rescue_bringup/launch/camera.launch.py` | Backend alternativo | V4L2/OpenNI2/Astra/Orbbec externos; índices/rutas fijos y contrato raw distinto del dashboard |
| `rescue_bringup/launch/logitech_vision.launch.py` | Excluir como receta mínima | Une vídeo y detector incondicional; `enable_yolo=false` no apaga Hazmat/AprilTags; índice default 2 |
| `rescue_bringup/launch/vision.launch.py` | Excluir | Une cámara + detector; `astra_core` sin profundidad por defecto, comentario OpenNI incorrecto; rutas modelos `/workspace` |
| `rescue_bringup/launch/slam.launch.py` | Excluir | Lidar/cámara opcionales, pero conversor depth, merger, SLAM, acumulador y GeoTIFF siguen arrancando; H07–H17 |
| `rescue_bringup/launch/full_bringup.launch.py` | Excluir | Perfil SLAM local; no inicia drivers de tracción Pi ni buses; seguir ejemplo de dos terminales duplica SLAM |
| `rescue_bringup/launch/lidar_ld19.launch.py` | Excluir del básico | Paquete externo; error `LDLidar communication KO` observado antes. Configura/activa con timers fijos, sin esperar éxito; `use_sim_time` declarado no pasado al componente |
| `rescue_bringup/launch/robot_description.launch.py` | Opcional para vídeo/conducción | Modelo RViz/TF; odom→base fijo, no odometría; sin articulaciones dinámicas del brazo/patas |
| `rescue_bringup/launch/save_map.launch.py` | Excluir | Guarda mapa, no lo crea; necesita `/map` real y nav2_map_server; no validado sin SLAM |

No hay una receta mínima Pi completa y validada que se arregle solamente cambiando
IP. La propuesta futura es componer **una vez** motor_driver + servos + publicadores
RGB directos; no usar `logitech_vision` como atajo para cámara sola.

## 6. YOLO, Hazmat y detecciones: inventario completo fuera del básico

| Elemento | Estado y motivo de exclusión |
|---|---|
| `rescue_bringup/rescue_bringup/object_detector.py` | Opcional; mezcla detección, localización, imágenes anotadas, markers y CSV; H08/H11/H13/H26/H29 |
| YOLO objetos `mission_objects_yolo.pt` | Peso presente; default absoluto `/workspace/src/...` falla en instalación nativa si no se sobreescribe; calidad/recall sin medir |
| YOLO auxiliar `yolov8n.pt` | Se intenta cargar para filtrar personas además del modelo de misión; puede descargar en runtime y añade inferencia; no necesario para transmitir vídeo |
| `hazmat_yolo.pt` | Presente; `hazmat_model` vacío no lo selecciona automáticamente, usa HSV; falta linaje y evaluación reproducible |
| `hazmat_yolo.onnx` | Presente; nodo lo carga mediante Ultralytics, no hay backend propio ORT/OpenCV-DNN que elimine esa dependencia |
| Hazmat HSV | Fallback de color/forma, no clasificación validada de sustancias; sigue activo aunque `enable_yolo=false` |
| AprilTags / pupil-apriltags | Detector independiente habilitado en perfiles de visión; fuera del control/vídeo básico |
| Filtros de clases, sinónimos y exclusión junto a personas | Heurísticas en detector; comentarios mezclan 49 y 13 clases; no inferir clases reales sin inspeccionar peso/evaluación |
| `training/train_hazmat.py` | Entrenamiento offline, no runtime; dataset `datasets/hazmat/data.yaml` ausente del checkout; copia resultado al modelo final |
| `training/export_to_onnx.py` | Exportador offline; default imgsz 416 frente a entrenamiento 640; no prueba paridad ni rendimiento en Pi |
| `training/test_hazmat_camera.py` | Prueba independiente de cámara/modelo; no valida ROS/DDS ni localización y compite por cámara si se ejecuta junto al driver |
| Entrenamiento de mission_objects | Comentarios remiten a entrenamiento que no está en `training/`; no hay receta reproducible local |
| `/object_detections`, `/object_detection_markers` | JSON y markers de detección; fuera del básico; no contrato con ID común para unir manuales/automáticos |
| `/camera/color/image_annotated/compressed` | Vídeo anotado opcional; no sustituye los canales RGB directos; salida fija aunque cambie cámara fuente |
| `/save_detection_csv` | Depende del detector; no existe proveedor si se excluye; dashboard conserva cliente/botón |
| Ultralytics/PyTorch, CLIP, modelos descargados, dependencias de exportación | Carga opcional de percepción/entrenamiento; Docker la instala aun cuando no se usa. No quitar librerías compartidas como OpenCV/NumPy |

Problemas funcionales adicionales del detector: no sincroniza color/depth/TF,
retiene profundidad vieja; con `require_depth=false` puede ubicar detecciones en
origen sin posición real; modo raw float falla con bridge local; dos detectores
producen registros/servicios ambiguos. Modelos presentes no significan modelos
cargados ni detección funcionando. No se cargaron pesos ni se entrenó en esta revisión.

QR del dashboard (`vision/qr_detector.py`) es otra implementación, no Hazmat ni
YOLO. Es una función opcional, pero sigue ejecutándose en el flujo de imagen y
su import `zxingcpp` es requisito de la UI actual. No se puede quitar esa
dependencia sin separar primero el consumidor.

## 7. SLAM 2D y «3D»: inventario completo fuera del básico

| Elemento | Qué hace / qué falta |
|---|---|
| `slam_toolbox`, `config/slam_toolbox_params.yaml` | SLAM 2D; necesita scans y TF coherentes. Nodo vivo no significa lifecycle activo o mapa actualizado |
| `ldlidar_component`, `config/ld19_params.yaml`, LD19 | Sensor externo; comunicación KO observada; no necesario para teleoperar con vídeo |
| `depthimage_to_laserscan` | Requiere Image raw + CameraInfo `/camera/depth/*`; no consume PNG comprimido del driver propio |
| `scan_merger.py`, `/scan_merged` | Fusiona scans; recorte no validado con yaw del lidar; tiempo cero/futuro y fallback TF, H07/H17 |
| odom→base estático | Identidad; no medición del avance. No hay productor de odometría física de orugas en esa ruta |
| `rescue_robot_description/urdf/rescue_robot.urdf.xacro` | Geometría/TF para visualización, no modelo articulado completo; no reemplaza calibración de brazo |
| `camera_drivers/point_cloud.py`, nube instantánea Astra | Proyección de profundidad; depende de profundidad métrica/calibración. No es SLAM |
| `pointcloud_accumulator.py`, `/accumulated_pointcloud` | Acumula nube usando TF `map`; no estima por sí mismo trayectoria 6D ni hace optimización SLAM 3D |
| `/camera/depth_registered/points`, `/camera/depth/points`, `/robot/camera/astra/points` | Entradas alternativas del acumulador; existencia de tópico no acredita datos ni TF |
| Exportación PLY `/save_pointcloud_ply` | Fuera del básico; H15/H16: formatos, fallback bloqueado tras color, límite de puntos y marco de exportación |
| `geotiff_writer.py`, `/save_geotiff` | Exporta imagen mapa/ruta/detecciones; TIFF de PIL sin georreferenciación GIS acreditada; H13–H15 |
| `nav2_map_server/map_saver_cli` | Guarda PGM/YAML; requiere mapa generado; independiente de GeoTIFF |
| RViz2 + `config/slam_rviz.rviz` | Visualización, no algoritmo de SLAM ni prueba de mapa correcto; puede arrancar desde launch o botón dashboard |
| `src/Slam/` | Directorios vacíos locales reportados antes; no implementación activa ni proveedor de SLAM 3D |
| Modos `slam`, `slam-pi`, `lidar`, `rviz`, `save-map`, `save-ply`, `save-geotiff`, `save-csv`, `save-mission`, `diag`, `scan` del script contenedor | Auxiliares de esos perfiles; innecesarios para control/vídeo básico |

No se encontró un launch/implementación local de SLAM 3D autónomo como parte del
stack. Una nube coloreada acumulada con la pose de SLAM 2D no acredita reconstrucción
3D con estimación de pose completa. Tampoco se auditó software fuera del checkout.

**Nuevo B04:** `COMO_EJECUTAR.md` conserva una llamada a `/save_pointcloud`, mientras
el acumulador ofrece `/save_pointcloud_ply`. Ese comando de la guía no coincide
con el proveedor local. El dashboard sí usa el nombre `_ply`.

## 8. Funciones del dashboard que sobran en el básico o quedan sin proveedor

- Inicio/parada de misión solo cambia estado y cronómetro de UI (H12).
- Guardar misión/CSV/PLY/TIFF requiere servicios de detector y mapas; desactivar
  esos nodos deja controles visibles pero sin resultados operativos.
- Detecciones manuales, geolocalización y ruta necesitan TF `map`; H13–H15.
- Abrir carpeta usa `/workspace/maps`, que puede diferir de la salida nativa.
- Botón RViz permite lanzar visualizador aunque `launch_rviz=false` en el padre.
- Suscripciones de profundidad/nube usadas solo para presencia B02; no son un visor 3D.
- Panel y anotación de QR/detecciones añaden procesamiento; opcionales para conducir.
- Mantener controles de patas, cambio de GUI, buses, telemetría y vídeo RGB: sí
  pertenecen al objetivo. `/arm_active` inhibe orugas al mostrar brazo por diseño;
  no confundir esa inhibición con un fallo de motores.

## 9. Scripts, instalación, alternativas y residuos

| Archivo/grupo | Clasificación y estado |
|---|---|
| `scripts/ros_net_pc.sh` | Red necesaria, receta desactualizada: IP/interface y rutas de usuario fijas; no se adapta a cualquier red |
| `scripts/ros_net_pi.sh` | Red necesaria, IP de peer/local fijas; ping a red histórica decide cable/Wi-Fi; no sigue DHCP automáticamente |
| `scripts/run_pi_sensors.sh` | Alternativa contenedor, no perfil básico: eth0/peers fijos, cámaras/lidar/servos, no motores; `-it` en servicio; H21 |
| `scripts/run_slam_container.sh` | Alternativa de despliegue con numerosos modos; red enp3s0/IP fija, rutas/paquetes históricos, credencial incrustada H18, copias parciales; no usar para sincronizar «todo» |
| `scripts/install_pi_autostart.sh` | Instalador de servicio, no driver; arranca receta anterior. Existencia del archivo no prueba servicio instalado; H21 |
| `scripts/habilitar_usb_ros.sh` | Soporte válido: grupos/permisos, requiere nueva sesión; no instala por sí solo todas las reglas/depencias |
| `99-pedros-rescue.rules` | Soporte necesario según hardware; AX depende de serial concreto FTDI; no cualquier U2D2; regla USB lidar no habilita UART GPIO |
| `Dockerfile`, `Dockerfile.pi` | Alternativas, no necesarios para ejecución nativa; instalan percepción adicional, dependencias repetidas, falta build/fijación de proveedores; H19 |
| `requirements_*.txt`, `system_requirements_*.txt`, `package.xml`, `setup.py` | Necesarios de instalación; no consistentes como una receta única; H19/H20. No borrar manifiestos por excluir un nodo |
| `src/dependencias/cv_bridge` | Alternativa local que puede ocultar bridge ROS; H08/H09; no retirar sin comprobar resolución de imports |
| `src/dependencias/joy` | Nodo de mando usado; tiene H04. No es sobrante solo por estar en dependencias |
| `rgbd_viewer_node.py` | Visor alternativo RGB-D, no iniciado por command_station; útil diagnóstico, innecesario para RGB básico |
| `dynamixel_sim_node.py`, `arm/simulator.py` | Simulación/pruebas, no drivers reales; conservar para desarrollo |
| `simulador_web_original/dist/*`, `verify*.cjs` | Referencia/herramientas; extract_arm_canvas usa parte de dist. No ejecutar todo el simulador original para controlar hardware; no borrar a ciegas |
| `arm/web_assets/*`, `arm/web_view.py` | Sí usados por GUI activa; conservar aunque sean JavaScript/HTML |
| `tools/extract_arm_canvas.py`, `verify_arm_canvas.py`, `verify_arm_gui.py`, `test/*` | Generación y validación; fuera de runtime, no fallos por no arrancarse |
| `docs/integracion_6r/*`, `ARM_6R.md`, manifiestos/hashes de entrega | Históricos/soporte; pueden describir geometría anterior; no prueba del estado actual |
| `docs/contexto_proyecto/*` | Contexto/evidencias; no runtime; algunos resultados describen el corte del 12 de septiembre |
| `build/`, `install/`, `log/`, `__pycache__`, `.vscode` | Derivados/locales/entorno; no borrar durante ejecución ni confundir con código muerto. `install` es lo que resuelve ROS |
| `resource/*`, `__init__.py`, `setup.cfg`, CMake/mensajes/servicios | Infraestructura de paquetes, necesaria aunque no aparezca como nodo |

Modos restantes del script contenedor: `build`/`rebuild`/`rebuild-pi` preparan
entornos; `exec`/`shell`/`topics` inspeccionan; `dashboard`/`arm` arrancan UI;
`camera-pi`/`camera` según script seleccionan cámaras; `logitech-vision`/`vision`/
`vision-test` añaden detección; `pi*` opera remotamente. Todos requieren revisar
la receta concreta. Los modos `arm` instalan dependencias y compilan, no son
un arranque ligero sin modificaciones del entorno.

## 10. Plan de corrección y aceptación para otro agente

1. Recuperar red real y preparar ambos peers/DDS; revisar procesos, servicios,
   dispositivos y fuente instalada en Pi antes de reiniciar. No reutilizar PIDs
   históricos de esta documentación para matar procesos.
2. Componer un solo dueño de motores y buses; separar cámara de detector y
   quitar anidamientos/flags ignorados. Conservar contratos PC↔Pi.
3. Corregir pérdida de joystick, comandos no finitos, caducidad de patas y
   detención/cancelación de brazo. Probarlo en banco seguro con feedback real.
4. Validar una cámara RGB a la vez, dispositivo estable, compresión y QoS; comprobar
   recepción en dashboard/brazo. Recién entonces dos cámaras simultáneas.
5. Para básico excluir detector, lidar, SLAM, nubes/exportadores y sustituir sus
   suscripciones de presencia por telemetría ligera; aislar controles de misión
   de UI o mostrar su indisponibilidad.
6. Documentar versiones PC/Pi y pruebas; mapas/percepción se habilitan después
   como perfiles independientes. No remover paquetes/pesos indiscriminadamente.

Aceptación todavía pendiente: una sola instancia por bus/controlador; joystick
desconectado conduce a cero; brazo responde con feedback y para/cancela de forma
comprobada; vídeo de cada cámara llega sostenidamente sin congelar SSH; reconexión
de red no conserva peers viejos; ninguna detección/mapa arrancando en el perfil
básico. No se ha superado esa aceptación en esta auditoría.

## 11. Cobertura de todos los hallazgos existentes

| Hallazgos | Secciones de esta revisión |
|---|---|
| H01–H05 | 3–5: arranque, cámaras, pérdida de mando/feedback |
| H06 | 1: corrección del alcance host vs aplicación |
| H07–H11 | 4, 6, 7: sensores/imagen/TF/detección |
| H12–H17 | 7–8: misión, datos, mapas y exportación |
| H18–H21 | 9: red, despliegue, instalación, autostart |
| H22–H25 | 2–3, 7–9: UI, control, unidades, geometría |
| H26–H29 | 4–6, 9–10: pesos, residuos, pruebas y detector incondicional |
| D01–D10 | 2–3, 9–10; detalle conservado en 08_DEPENDENCIAS_CRUZADAS.md |

Los B01–B05 son adiciones de esta revisión; no reemplazan ni renumeran los H.
Para reproducción/evidencia histórica detallada: [02_HALLAZGOS.md](02_HALLAZGOS.md)
y [06_COMPROBACIONES_Y_LIMITES.md](06_COMPROBACIONES_Y_LIMITES.md).
