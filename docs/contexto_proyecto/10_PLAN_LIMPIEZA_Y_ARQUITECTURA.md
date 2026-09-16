# Plan de limpieza y arquitectura independiente PC/Pi

Estado: **plan solicitado, no implementado**. Fecha: 2026-09-15.
Referencia del código: `ea41b28284ac066ae95b3d2642533453d348daae`.
Inventario y fallos: [09_PERFIL_BASICO_Y_EXCLUSIONES.md](09_PERFIL_BASICO_Y_EXCLUSIONES.md).

## 1. Objetivo y decisiones

Entregar un sistema mínimo de conducción de orugas, patas, brazo 6R y vídeo RGB
frontal/Astra, instalable y comprobable por separado en PC y Raspberry.
Retirar de la rama básica percepción automática, mapas y recetas redundantes.

**Decisión expresa del usuario:** el PC conoce apodos lógicos. Envía `Hombro`;
la Pi traduce `Hombro` al bus, ID y calibración físicos. No trasladar al PC una
copia de `servos.yaml` ni esconder la dependencia bajo otro nombre de paquete.

Se conservarán como claves canónicas los nombres actuales, sensibles a mayúsculas:
`Base`, `Hombro`, `Codo`, `Munieca_P`, `Munieca_Y`, `Munieca_R` y las cuatro patas
`PataDelIzq`, `PataDelDer`, `PataTrasIzq`, `PataTrasDer`. La UI puede mostrar
«Muñeca» con acento sin cambiar la clave transmitida. Un alias visual no cambia
la identidad lógica. Cambiar ID físico de Hombro debe requerir solo configuración
y reinicio controlado de Pi, sin recompilar o editar el PC.

Mantener un repositorio y separar paquetes/perfiles, no clonar contratos ni
crear dos repositorios que diverjan. La independencia significa que PC no
instala/importa hardware Pi, y Pi no instala/importa GUI PC. Ambos dependen de
contratos compartidos neutrales; la operación real requiere comunicación.

## 2. Arquitectura objetivo

```text
PC
  rescue_command_station: UI, mando, FK/IK, trayectorias, cliente del robot
    ├── rescue_interfaces: mensajes/servicios versionados
    └── rescue_robot_model: nombres, geometría y límites mecánicos nominales

Pi
  rescue_robot_core: admisión de órdenes, traducción lógica, drivers, cámaras
    ├── rescue_interfaces
    └── rescue_robot_model

Pruebas opcionales PC
  rescue_simulation: implementa el mismo contrato público sin GPIO/SDK
    ├── rescue_interfaces
    └── rescue_robot_model
```

No crear un nuevo bringup común que vuelva a depender de ambas aplicaciones.
Los launches públicos serán propiedad de cada paquete:

- `rescue_command_station/launch/pc.launch.py`.
- `rescue_robot_core/launch/pi.launch.py`.
- `rescue_simulation/launch/simulation.launch.py`.

Los nombres nuevos son objetivos del plan, todavía no existen. FK/IK y planificación
se quedan en PC. Pi valida y ejecuta consignas articulares; no necesita importar
el solver del PC. La validación de Pi y el protocolo de parada no dependen de
que el dashboard siga vivo.

### Reglas de dependencia comprobables

| Paquete/perfil | Dependencias locales permitidas | Prohibidas |
|---|---|---|
| `rescue_interfaces` | Ninguna aplicación | core, station, simulación, GUI, SDK |
| `rescue_robot_model` | Recursos puros; no nodos | core/station, serial, ROS de ejecución, GUI |
| `rescue_command_station` | interfaces, model | core, bringup antiguo, imports de simulación en runtime real |
| `rescue_robot_core` | interfaces, model | station, simulación, Tk/Chromium/solver PC |
| `rescue_simulation` | interfaces, model | core, SDK, GPIO, archivos privados de Pi |

Comprobar imports, `package.xml`, `setup.py`, búsquedas ament, rutas en scripts,
includes launch y dependencias de pruebas. Quitar solo una etiqueta XML no basta.

## 3. Contrato lógico del robot

### 3.1. Ejemplo de traducción

```text
PC: mover {joint: Hombro, position: 0.4 rad, speed_scale: 0.25}
Pi: validar Hombro, límites, sesión, secuencia, caducidad y estado
Pi: configuración local → bus EX-106+, id 1, reducción 25, cero/signo calibrados
Pi: adaptar radianes lógicos a representación del driver
Pi → PC: {joint: Hombro, position: medida, ready: true/false, fallo: ...}
```

ID 1 y reducción 25 provienen de la configuración actual; no se certifica su
calibración física. Esos valores no entran en el mensaje del PC.

### 3.2. API objetivo, a especificar en `rescue_interfaces`

| Canal propuesto | Datos mínimos / semántica |
|---|---|
| `/robot/info` | Versión de protocolo, boot/session ID, revisión del modelo, nombres disponibles, capacidades y límites efectivos; consulta o estado con durabilidad apropiada |
| `/robot/drive/command` | Izquierda/derecha normalizadas [-1,1], secuencia, sesión y vigencia; no fingir m/s en un Twist |
| `/robot/drive/state` | Salidas aplicadas, conexión, caducidad y fallo; velocidad medida solo si existe sensor |
| `/robot/arm/command` | Lista de articulaciones, posiciones en radianes, escala de velocidad [0,1], secuencia/sesión/vigencia |
| `/robot/arm/state` | Estado agregado por nombre, posición medida, validez/frescura, disponibilidad, emergencia y error; sin exigir IDs físicos al cliente |
| `/robot/legs/command` | Nombres, sentido y escala normalizada, secuencia/sesión/vigencia; definir si acepta comando parcial |
| `/robot/legs/state` | Nombres, posición/ángulo con unidad explícita, validez, disponibilidad y error |
| `/robot/stop` | Solicitud explícita de parada, alcance y acuse del estado real; idempotente |
| `/robot/enable` | Habilitación explícita tras conexión/emergencia; no reejecutar objetivos antiguos |
| Servicios lógicos de mantenimiento | Conectar, desconectar, consultar, calibrar por nombre o grupo; sin elección AX/EX ni ID desde GUI normal |
| Vídeo | Mantener nombres actuales `/robot/camera/front/image_raw/compressed` y `/robot/camera/astra/color/image_raw/compressed`, JPEG, BEST_EFFORT, cola corta |

Definir mensajes tipados para comandos nuevos; no usar `JointState.velocity`
como porcentaje. Se puede conservar `JointState` como feedback SI, agregando un
estado de salud aparte, siempre que no se mezclen mediciones frescas y obsoletas.
La UI trabaja en grados si conviene; convierte una sola vez al entrar/salir del
contrato. Conservar y probar la transformación `model_sign` del solver actual:
primero documentar signo/cero lógico por junta, no cambiar silenciosamente el
sentido del robot al pasar de grados a radianes.

Decisión inicial: órdenes del brazo con el vector completo de seis juntas para
movimiento coordinado. Mantenimiento por una junta usa servicio explícito.
Rechazar una orden completa si cualquier junta es desconocida, duplicada, no
disponible o fuera de límites; no mover silenciosamente solo las que funcionan.
La escritura sobre dos buses no es atómica físicamente: si falla un despacho,
detener el grupo afectado y reportar estado parcial/fallo, no responder éxito total.

### 3.3. Un único punto de admisión en Pi

Crear `rescue_robot_core/control/robot_controller_node.py` y traductor local
`hardware_mapping.py` (nombres propuestos). Es el dueño de conexión, validación,
arbitraje, caducidad, cancelación y reporte público. Los buses son implementaciones
internas; preferir clases/hilos internos con una apertura por puerto, sin topics
públicos que permitan saltarse la admisión. No basta renombrarlos a `private`:
un namespace ROS no impide que otro cliente publique allí.

Durante transición se pueden adaptar drivers existentes por interfaces Python
internas. Eliminar sus suscripciones/servicios externos antiguos al terminar;
no ejecutar a la vez el adaptador y la ruta antigua de control sobre el mismo bus.
La lógica de tiempo límite debe ejecutarse aunque las lecturas fallen. Caducidad
por reloj monotónico local; secuencia y sesión previenen replay/reordenamiento.
Si se usa timestamp de origen para edad de transporte, definir sincronización
y tolerancias; no comparar ciegamente relojes de dos máquinas.

Parada de orugas y patas: mandar cero de forma local ante pérdida de cliente.
Brazo: definir y probar por articulación una parada que detenga la trayectoria
sin caída por gravedad. No traducir genéricamente «parar» a «torque off» del
hombro. Antes de liberar hardware, documentar cuál es el comportamiento real
del dispositivo ante pérdida de lectura, corte de comunicación y emergencia.

### 3.4. Mantenimiento, compatibilidad y simulación

La GUI actual llama `/ax12a/*`, `/ex106/*`, envía `ServoCommand.id` y consulta
`ServoStatus.ids`. Migrar también esas acciones; no limitar la refactorización
al envío de trayectorias. Registrar un ID/cambiar bus queda como herramienta
local de mantenimiento Pi con actuadores detenidos, no dependencia del PC.
Una UI puede mostrar diagnóstico técnico opcional suministrado por Pi, pero no
usarlo para decidir el destino de una orden normal.

El PC arranca desconectado y permite simulación sin Pi. Al conectar valida
versión, lista de nombres y modelo; bloquea movimiento ante incompatibilidad.
Pi nunca adopta automáticamente calibraciones enviadas por el PC. El simulador
responde al mismo contrato lógico y no tiene mapa físico de servos.

La migración rompe protocolos: declarar versión 2. Desplegar PC/Pi compatibles
en una ventana de mantenimiento; impedir mezcla de control v1/v2. Las APIs
cartesianas internas del PC pueden mantenerse mientras sus clientes concuerden.

## 4. Propiedad de configuración y archivos

| Origen actual | Destino/acción | Condición |
|---|---|---|
| `station/config/arm.yaml` | `rescue_robot_model/config/arm.yaml`: geometría, orden, límites nominales, signos lógicos | Una fuente de modelo; conservar L3=0.05/herramienta=0.08 hasta recalibrar |
| `core/config/servos.yaml` | Privado Pi: mapa nombre→bus/ID, puerto/baudrate, reducción, signo físico, cero, límites efectivos | Conservar datos actuales, distinguir provisionales y verificar en banco |
| `station/arm/configuration.py:joint_drivers` | Eliminar lectura de core; consumir nombres/modelo y capacidades de Pi | Migrar primero todos los clientes |
| `station/arm/configuration.py:settings` | Cargar model neutral; resolver override antes del default instalado | Probar fuera del checkout con perfil PC solo |
| `core/servos/params.py`, `wheel_encoder.py` | Permanecen privados Pi | Simulación no los importa después |
| `core/nodes/dynamixel_sim_node.py` | Reimplementar como proveedor lógico en `rescue_simulation` | Trasladar lógica pura necesaria sin importar core; no copiar parser hardware completo |
| `station/arm/kinematics.py`, `motion.py`, `cartesian_controller.py` | Conservar PC | Refactorizar fronteras, no reemplazar motor matemático validado |
| `station/arm/cartesian_node.py` | Cliente del contrato lógico; eliminar partición AX/EX y lectura de IDs | Enviar una orden, consumir estado agregado y confirmar parada |
| `station/arm/gui_node.py` | Presentación de nombres/estado y servicios lógicos | Quitar diccionarios driver/sid y botones bus específicos |
| `station/nodes/dashboard_node.py` | UI de conducción/patas/vídeo, sin misión/TF/mapas ni subprocess | Arranque de nodos pasa al launch PC |
| `station/control/*`, `input/ps4_controller.py` | Conservar y probar mezcla/deadzone | Adaptar salida al contrato drive; confirmar mapping PS4 |
| `core/camera_drivers/logitech_camera_node.py` | Base preferida de publicador RGB con reapertura | Verificar dispositivo estable MJPG y recuperación |
| `core/camera_drivers/astra_rgbd_camera_node.py` | Extraer RGB a publicador dedicado, sin falsa profundidad | Backend UVC confirmado para cámara real antes de borrar alternativa |
| `core/camera_drivers/ros_image.py` y `station/vision/ros_image.py` | Mantener solo conversiones necesarias | Emisor/receptor separados son válidos; pruebas JPEG/mensajes inválidos |
| `station/arm/web_assets/*`, `web_view.py` | Conservar | Activos reales de la GUI; no confundir con percepción |

Aquí `station` abrevia `src/rescue_command_station/rescue_command_station`
para módulos y `src/rescue_command_station` para config/launch; `core` sigue el
mismo criterio con `rescue_robot_core`. En la implementación usar rutas exactas.

## 5. Matriz de retirada del árbol activo

Retirar tras los reemplazos indicados, en commits recuperables. No borrar el
historial Git ni modelos históricos remotos; no es necesario crear una carpeta
`legacy` instalada que mantenga dependencias vivas.

| Ruta exacta/grupo | Acción objetivo | Antes de eliminar |
|---|---|---|
| `src/rescue_bringup/rescue_bringup/object_detector.py` | Borrar del básico | Quitar clientes, imágenes anotadas, CSV y detecciones de ambas GUIs |
| `src/rescue_bringup/models/{hazmat_yolo.pt,hazmat_yolo.onnx,mission_objects_yolo.pt}` | Borrar del árbol básico | Registrar commit de recuperación y retirar data_files/referencias |
| `training/` (tres scripts actuales) | Borrar del árbol básico | Sin consumidores runtime; recuperar desde Git si vuelve percepción |
| `src/rescue_bringup/rescue_bringup/{scan_merger,pointcloud_accumulator,geotiff_writer}.py` | Borrar | Retirar sus launches, servicios/clientes/UI/configuraciones |
| `src/rescue_bringup/rescue_bringup/logitech_pub.py` | Borrar variante redundante | RGB directo Pi probado, incluyendo reconexión |
| `src/rescue_bringup/config/{ld19_params.yaml,slam_toolbox_params.yaml,slam_rviz.rviz}` | Borrar | Sin consumers lidar/mapa/RViz en básico |
| `src/rescue_bringup/launch/{slam,full_bringup,lidar_ld19,save_map,vision,logitech_vision}.launch.py` | Borrar | UI y documentación migradas |
| `src/rescue_bringup/launch/{pedro_pc,pedro_pi,pi_sensors}.launch.py` | Sustituir por launches propietarios y borrar | Nuevos PC/Pi superan prueba de arranque sin duplicados |
| `src/rescue_bringup/launch/camera.launch.py` | Borrar selección multi-driver heredada | Ambos canales RGB tienen backend verificado |
| `src/rescue_bringup/launch/robot_description.launch.py` | Retirar del runtime básico | Ningún consumidor TF/mapa restante; cámara RGB no requiere mapa |
| `src/rescue_bringup/` restante (setup, resource, package, init) | Borrar paquete completo | Todas sus funciones necesarias migradas; cero referencias activas |
| `src/rescue_robot_description/` | Retirar del básico con el URDF histórico | Confirmar que GUI usa su modelo Canvas y no depende de xacro/TF; modelo nuevo neutral es separado |
| `src/rescue_robot_core/rescue_robot_core/camera_drivers/point_cloud.py` | Borrar | Extraer RGB y eliminar import/publicación de nubes |
| `src/rescue_command_station/rescue_command_station/nodes/rgbd_viewer_node.py` | Retirar visor RGB-D del básico | Quitar console_script; conservar vistas RGB activas |
| `src/rescue_command_station/rescue_command_station/vision/qr_detector.py` | Retirar QR del perfil mínimo | Quitar import/callback de dashboard y dependencia zxing-cpp |
| `src/dependencias/cv_bridge/` | Borrar implementación local | Ningún consumidor restante; si se necesita bridge, usar proveedor ROS explícito |
| `src/dependencias/joy/` | Reemplazar por nodo de entrada PC con estado real de conexión | Reproducir H04 y corregirlo en nuevo nodo; no confiar solo en cambio a driver oficial |
| `scripts/run_slam_container.sh`, `scripts/run_pi_sensors.sh` | Borrar recetas multipropósito | Instalación nativa y arranques nuevos documentados |
| `scripts/install_pi_autostart.sh` | Reemplazar por servicio Pi del perfil básico | Quitar servicio antiguo solo tras identificar si existe; probar apagado/reinicio |
| `scripts/ros_net_pc.sh`, `scripts/ros_net_pi.sh` | Reescribir wrappers del perfil local | Configuración explícita validada, sin direcciones/rutas de usuario fijas |
| `Dockerfile`, `Dockerfile.pi` | Retirar recetas antiguas del básico | Elegir instalación nativa como fuente principal; contenedor futuro usa mismos manifiestos |
| `simulador_web_original/` bajo station | Conservar durante migración | Solo retirar después de independizar generador y conservar activos/procedencia; no borrar dist requerido |
| `src/Slam/`, build/install/log/caches | Inspeccionar residuos locales por separado | No versionados; limpiar solo rutas verificadas fuera de sesiones activas; no usar borrado global |

No considerar descartables los fallos en drivers, control, codificación JPEG o
calibración: esas piezas necesarias se reparan/reemplazan. No borrar tests porque
fallan; conservar reproducciones y adaptarlas al contrato nuevo.

## 6. Interfaces y dependencias que se retiran con sus consumidores

`ServoCommand.srv`, `ServoStatus.srv`, `RegisterServo.srv` actuales exponen IDs.
Crear reemplazos lógicos, migrar todos los clientes/servidores/tests y después
retirar definiciones v1 de CMake. `ArmStatus.msg` puede requerir sustitución por
estado por nombre. Mantener `ComputeIKPose`, `CartesianGoto`,
`CartesianTrajectory`, `CartesianWaypoint` y `CartesianState` mientras sirvan
al PC; verificar unidades y compatibilidad. No borrar todo `rescue_interfaces`.

Eliminar de requirements/manifiestos/Docker retirados: Ultralytics/PyTorch/CLIP,
ONNX Runtime si solo servía percepción, pupil-apriltags, zxing-cpp tras quitar QR,
slam_toolbox, nav2 map/lifecycle, RViz, lidar, depthimage_to_laserscan, TF/mapas y
mensajes de visualización cuando la búsqueda de consumidores quede vacía.
OpenNI2/Orbbec SDK solo se retiran si RGB real no los necesita. Mantener
OpenCV/NumPy/PIL/Tk/CustomTkinter/Playwright/Chromium según imports activos; no
confundir Pillow para UI con Pillow usado por GeoTIFF. Evaluar matplotlib por
consumidores del simulador antes de quitarlo.

No desinstalar globalmente paquetes apt/pip del equipo como parte de limpiar
el repositorio: construir perfiles limpios verifica lo que necesitan sin romper
otras aplicaciones. Fijar Jazzy/Python 3.12 en recetas, no placeholders ambiguos.

## 7. Fases ejecutables, entregables y puertas de salida

### F0 — Capturar estado y preparar recuperación

- Registrar commit y `git status`, inventario de paquetes/scripts/modelos,
  proceso de instalación actual y copia privada de calibración real Pi.
- Inspeccionar fuente y `install` remotos, puertos, cámaras, systemd y procesos.
  No asumir que Pi ejecuta el mismo commit por tener carpeta del mismo nombre.
- Crear rama de implementación y referencia de recuperación; conservar los
  cambios del usuario. Medir situación inicial con pruebas sin movimiento.
- Salida: mapa de despliegue actual y lista exacta de recursos físicos confirmada.

### F1 — Definir contrato y modelo lógico

- Crear `rescue_robot_model`, mover modelo y definir API v2 en interfaces.
- Especificar nombres, unidades, límites, errores, parada, sesión/reconexión,
  caducidad y comportamiento ante órdenes parciales/duplicadas.
- Crear proveedor falso lógico que permita continuar PC sin Pi.
- Salida: ejemplos de orden/feedback y tests que validan esquema, unidades y
  rechazo de junta desconocida. PC no necesita un ID físico para esos ejemplos.

### F2 — Pi: traducción y ejecución segura

- Implementar admisión única y mapa privado, extraer/adaptar drivers internos.
- Resolver H05/H23/H24: validación finita, lectura fallida, watchdog independiente,
  cancelación y fallo parcial de buses. Mapas inválidos abortan antes de torque.
- Exponer estado agregado y mantenimiento lógico; quitar autoconexión parcial
  del PC: Pi administra cada recurso y publica cuándo está listo.
- Salida: con buses falsos `Hombro` llega al ID configurado; cambiar ID solo en
  Pi preserva API; ningún camino antiguo puede mandar por fuera de admisión.

### F3 — PC: migrar clientes y propiedad de procesos

- Eliminar `joint_drivers`, IDs/buses de GUI y partición AX/EX de cartesian_node.
- Extraer cliente robot de GUI; mantener FK/IK y arbitraje de entrada PC.
- Nuevo launch PC inicia mando, teleop, cinemática, cartesiano, GUI brazo y
  dashboard una sola vez. UI cambia visibilidad, no crea procesos con Popen.
- Nodo de joystick publica conexión real; EOF/error desconecta, borra estado y
  no entrega ejes antiguos como eventos nuevos. Definir inhibición drive/arm/patas.
- Salida: PC solo arranca con falso; pérdida de mando da cero y bloquea orden
  nueva hasta reconexión válida. Cierre termina hijos incluso con GUI oculta.

### F4 — RGB mínimo y red

- Elegir y comprobar dispositivo por identidad estable, no `/dev/videoN` supuesto.
- Publicador RGB por cámara, cola corta, compresión, reapertura y estado ligero;
  evitar lectura bloqueante que impida apagar. No abrir el mismo canal dos veces.
- Dashboard consume solo RGB y estado; quitar nubes/depth como heartbeat,
  detección/QR/TF y controles de misión. Mantener vídeo de brazo.
- Configuración local externa para interfaz/IP/peer/domain; scripts obtienen
  rutas relativas al checkout o instalación, imprimen configuración efectiva y
  fallan claramente si interfaz/IP no corresponde. No prometer cambios DHCP
  transparentes: documentar resolución/reinicio del proceso al cambiar red.
- Salida: dos cámaras transmiten y reconectan sin bloquear el control; métricas
  de CPU/red/fps/latencia permiten diagnosticar pérdidas de enlace.

### F5 — Instalación y simulación independientes

- Eliminar dependencias cruzadas de package.xml/setup/launch después de migrar
  consumidores; añadir dependencias realmente usadas (SDK/serial/ament/etc.).
- Crear listas explícitas de paquetes por perfil y recetas sin includes de
  workspace ajeno. Usar solo `/opt/ros/jazzy` como underlay de pruebas.
- Separar pruebas numéricas de PC de pruebas de drivers; simulación consume API
  lógica, sin importar `leer_servos_params` ni `WheelEncoder` de core.
- Salida: instalación limpia de PC sin core/SDK/GPIO; Pi sin station/Tk/Chromium;
  simulación sin hardware. Los dos extremos comparten versión interfaces/model.

### F6 — Borrar extras y cerrar referencias

- Aplicar matriz de retirada, quitar entry points/data_files/clientes/UI/tests
  específicos de extras o archivar su historia en Git según corresponda.
- Borrar bringup monolítico solo al final de la migración de arranque/cámara.
- Verificar recursos instalados: un overlay viejo puede ocultar referencias rotas.
- Actualizar README, COMO_EJECUTAR, requisitos, documentación y servicios locales
  aplicables. Contexto histórico permanece marcado como histórico.
- Salida: cero referencias ejecutables a componentes retirados y build limpio.

### F7 — Banco y despliegue

- Comparar contratos/modelo y calibración, instalar ambos lados coordinadamente.
- Ensayar parada antes de desplazamientos normales; alimentación/carga apropiadas.
- Probar cada junta y pata por nombre, luego movimiento coordinado, y vídeo
  simultáneo con control. Medir desconexión/reconexión, SSH y consumo.
- Servicio Pi tiene un solo dueño del arranque, sin TTY ni instalaciones pip en
  inicio; impedir segunda instancia por bloqueo real de recurso/proceso.
- Salida: tabla de aceptación completada con resultados, duración y evidencia.

## 8. Verificación obligatoria y criterios medibles

| Prueba | Resultado exigido |
|---|---|
| Instalación PC limpia | Abre dashboard/brazo sin core ni Pi; no busca config/SDK de Pi |
| Instalación Pi limpia | Inicia control y RGB sin station/Tk/Chromium ni pesos de detección |
| Cambio físico Hombro | Solo se modifica mapa Pi; PC sigue enviando `Hombro`, sin IDs |
| Esquema/modelo incompatible | UI explica incompatibilidad y bloquea movimiento |
| Bus retrasado/no disponible | Estado por articulación correcto; no falso «todo conectado» |
| Mando desconectado/EOF | Cero local y órdenes caducadas; no timestamps que oculten desconexión |
| NaN/Inf, nombres repetidos, fuera de límites | Rechazo sin escrituras parciales a actuadores |
| Lectura de servo falla | Watchdog sigue ejecutándose; fallo reportado y paro físico verificado |
| Cancelación/emergencia | Confirmación del estado de parada; prueba de retención segura del hombro |
| Reinicio PC/Pi | Nueva sesión; no replay de último objetivo; habilitación controlada |
| GUI cerrada/reabierta | Un solo controlador; ningún hijo huérfano dueño del bus |
| Cámara desconectada y reconectada | Estado offline claro, recupera frames; control sigue operativo |
| Dos RGB + control sostenidos | Ensayo propuesto ≥10 min; registrar fps, latencia, CPU, red, errores y disponibilidad SSH |
| Cortes de red | Parada dentro del límite acordado en F1; no reanudar automáticamente órdenes caducadas |
| Perfil básico limpio | Sin YOLO/Hazmat/SLAM/lidar/nubes/exportadores, ni sus suscripciones/servicios |

Fijar en F1 límites temporales numéricos por actuador basados en banco; no
certificar como seguros los defaults actuales (motor 1 s, patas 0.4 s, Joy 0.7 s)
sin medir. Guardar la evidencia de cada test en contexto; unit tests no prueban
torque, cableado, frenado ni imágenes reales.

Ejemplos de búsquedas de cierre (revisar resultados, no borrar automáticamente):

```bash
rg -n 'rescue_robot_core|rescue_bringup|/ax12a/|/ex106/|joint_drivers|servos_config_path' src/rescue_command_station
rg -n 'rescue_command_station|customtkinter|playwright|tkinter' src/rescue_robot_core
rg -n 'object_detector|hazmat|ultralytics|slam_toolbox|scan_merger|pointcloud_accumulator|geotiff_writer|save_detection_csv' src scripts requirements* system_requirements*
```

Resultados en documentación histórica son aceptables; resultados en imports,
manifiestos, launches y runtime básico exigen explicación o corrección.

## 9. Secuencia de commits y recuperación

Un commit revisable por fase: baseline → contrato/modelo → Pi → clientes PC →
RGB/red → perfiles/simulación → eliminación → validación/documentación.
No mezclar retirada masiva con cambios de signo/calibración. Si falla una puerta,
no seguir borrando dependencias que la prueba todavía usa.

Recuperar versiones compatibles completas de PC/Pi y configuración local;
no hacer checkout parcial de un driver sobre otro protocolo. Datos/modelos
retirados siguen recuperables desde el commit previo. Borrar archivos del árbol
no reduce por sí solo el historial/tamaño remoto de Git.

## 10. Decisiones pendientes antes de hardware, no bloqueantes para este plan

La propuesta del usuario sobre apodos ya queda adoptada. Confirmar durante F0/F1:

- Identidad física y canal RGB de cada cámara; qué cámara ve la GUI del brazo.
- Calibración real/firmware de cada articulación y política de parada con gravedad.
- Límites medidos de caducidad/latencia y si el cliente requiere control simultáneo
  de orugas/brazo (por defecto conservar exclusión actual).

Por defecto se conserva control de patas y mantenimiento/calibración por nombre;
se retiran QR, Hazmat, mapas y nubes del perfil básico. Cambiar esas decisiones
requiere ajustar la matriz y aceptación antes de implementar. Este documento
es el encargo técnico para el siguiente agente; todavía no se autoriza ni realiza
un borrado en esta tarea de planificación.
