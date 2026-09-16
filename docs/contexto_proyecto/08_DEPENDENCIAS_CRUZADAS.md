# Dependencias cruzadas y separación entre PC y robot

## Plan de implementación acordado — apodos lógicos

El usuario pidió un plan detallado y estableció que el PC conozca `Hombro`,
mientras la Pi resuelve su bus/ID físico. El plan vigente está en
[10_PLAN_LIMPIEZA_Y_ARQUITECTURA.md](10_PLAN_LIMPIEZA_Y_ARQUITECTURA.md):
contrato por nombres, configuración privada Pi, simulación independiente,
matriz de retirada, fases, pruebas y recuperación. Incluye migrar servicios
de mantenimiento y feedback; no solo el tópico de movimiento.

Las propuestas generales siguientes son antecedentes. La decisión vigente
descarta copiar el mapa de buses/IDs al PC o moverlo a un paquete compartido.
Se comparten contratos y modelo lógico, no detalles de hardware. Plan pendiente
de implementación; no se han borrado componentes por esta solicitud.

## Auditoría operativa del perfil básico

El inventario de lo necesario, lo que falla y lo excluible para movimiento,
brazo y cámaras está en [09_PERFIL_BASICO_Y_EXCLUSIONES.md](09_PERFIL_BASICO_Y_EXCLUSIONES.md).
Incluye todos los launches locales, YOLO/Hazmat, SLAM 2D, nubes 3D, exportación,
scripts y limitaciones de hardware. Las IP de la nota siguiente son históricas:
en la revisión posterior el PC volvió a `172.23.12.235/23`, con el launch aún
configurado para la red anterior. Consultar direcciones reales antes de arrancar.

## Nota operativa de red — 2026-09-15

Durante la ejecución real de esta sesión, el PC quedó en `10.230.234.1/24` por
`wlan0` y la Raspberry `gardian` en `10.230.234.137/24` por `wlan0`. El nombre
mDNS `gardian.local` permitió entrar por SSH aunque la Raspberry hubiese
cambiado de dirección.

CycloneDDS está configurado en unicast, por lo que no basta con que ambos
equipos estén conectados a la misma red: cada launch debe recibir la IP propia
y la IP del peer. La correspondencia obligatoria es:

```text
pedro_pi.launch.py: pc_wifi_ip=<IP_PC>  pi_wifi_ip=<IP_PI>
pedro_pc.launch.py: pi_wifi_ip=<IP_PI>  pc_wifi_ip=<IP_PC>
```

En esta sesión se lanzaron con:

```bash
# Pi
ros2 launch rescue_bringup pedro_pi.launch.py network:=wifi \
  wifi_interface:=wlan0 pc_wifi_ip:=10.230.234.1 pi_wifi_ip:=10.230.234.137

# PC
ros2 launch rescue_bringup pedro_pc.launch.py network:=wifi \
  wifi_interface:=wlan0 pi_wifi_ip:=10.230.234.137 pc_wifi_ip:=10.230.234.1
```

Si las IP cambian, consultar `ip -4 -brief address` en cada equipo y
`getent hosts gardian.local` desde el PC. Los lugares que contienen defaults o
valores fijos son:

- `src/rescue_bringup/launch/pedro_pi.launch.py`: `pc_wifi_ip`, `pi_wifi_ip`,
  `wifi_interface`.
- `src/rescue_bringup/launch/pedro_pc.launch.py`: `pi_wifi_ip`, `pc_wifi_ip`,
  `wifi_interface`.
- `scripts/ros_net_pi.sh`: `_IF`, `_PEER`, `_SELF`.
- `scripts/ros_net_pc.sh`: `_IF`, `_PEER`, `_SELF`.

Para una ejecución puntual es preferible pasar los valores al launch, sin
editar los defaults. No se guardan credenciales SSH en el repositorio.

Revisión ampliada por Codex: 2026-09-12, commit base `3b755cec99230810fd10f5776eb4522d82439095`, con los cambios locales en requirements_pc.txt y arm.yaml preservados. Solo revisión/documentación; no se cambiaron paquetes, launch ni código. Esta revisión rectifica afirmaciones de la versión anterior de este documento y conserva los IDs D01–D08.

## Respuesta a la pregunta del usuario

PC y robot deberían poder **instalarse, arrancar y probarse por separado**, sin que el PC necesite los drivers reales de la Raspberry ni la Raspberry necesite el dashboard, Tkinter o Chromium. En ese sentido la separación que propone el usuario es correcta.

Ambos forman un sistema distribuido y necesitan acordar mensajes, servicios, nombres de articulaciones, unidades, marcos de coordenadas y comportamiento ante desconexión. Compartir ese contrato, y bibliotecas puras cuando sean útiles, es sano. Duplicar esos contratos en dos implementaciones «totalmente independientes» produciría precisamente dos fuentes de verdad.

No hace falta separar repositorios: un solo repositorio puede contener paquetes y perfiles de instalación independientes. Tampoco instalar un paquete significa ejecutar sus drivers o necesitar físicamente una Raspberry para compilar Python. El defecto actual es que algunas dependencias de instalación y recursos están agrupadas por conveniencia y cruzan la frontera funcional.

| Independencia | Qué debería significar | Estado observado |
|---|---|---|
| Código PC/robot | Ningún lado importa la implementación específica del otro | Los imports Python de producción ya lo respetan entre station y core; ambos importan interfaces. |
| Instalación | Cada perfil incluye únicamente los paquetes y dependencias que usa | Incumplido por station→core y bringup común→station/core. |
| Arranque | PC puede mostrar desconectado; robot puede arrancar sin GUI y quedar seguro | Hay búsquedas de recursos que exigen core instalado en PC y launch mezclados. Reacción física sin PC no validada. |
| Pruebas | Lógica PC con transporte/simulación; robot con clientes de prueba sin dashboard | Algunas pruebas puras existen, pero el simulador ROS está empaquetado con hardware. |
| Protocolo | Ambos entienden el mismo contrato compatible | rescue_interfaces es la separación correcta; hay unidades/IDs/JSON/nombres con acoplamiento implícito. |
| Operación | Un lado puede perder conexión sin que el otro quede en estado peligroso | H04/H05/H23 impiden dar esto por garantizado. No equivale a seguir toda misión sin PC: actualmente la planificación/ejecución cartesiana de alto nivel vive allí. |

## Evidencia del grafo actual

Extracción AST de imports absolutos y llamadas de recursos, más lectura de los siete package.xml. Se excluyeron tests/tools del grafo de imports de producción. Resultado completo en [dependencias_cruzadas_evidencia.json](dependencias_cruzadas_evidencia.json). No ejecuta código de nodos.

Dependencias locales declaradas relevantes:

```text
rescue_bringup ──> rescue_command_station ──> rescue_robot_core
       │                    │                        │
       ├──> rescue_robot_core                        │
       ├──> rescue_robot_description                 │
       └──> cv_bridge       └──> rescue_interfaces <──┘
                            └──> joy
```

No hay ciclo en las dependencias locales declaradas. Sí hay una dependencia de recursos de station hacia bringup que cierra un ciclo conceptual y que se convertiría en ciclo declarado si se añadiese directamente al manifiesto.

Imports entre paquetes detectados: bringup→cv_bridge, station→interfaces y core→interfaces. **No se encontraron imports de drivers core desde station ni de GUI station desde core** en el código de producción analizado. Las referencias en comentarios no cuentan como imports. Las búsquedas ament, archivos y nodos launch sí crean dependencias adicionales aunque no exista import Python.

## D01 · Confirmado y precisado · El empaquetado mezcla dominios

Evidencia: `src/rescue_bringup/package.xml:13–15`, `src/rescue_command_station/package.xml:25`. Bringup declara station/core/description; station declara core. Core no declara station ni bringup. Interfaces y description no dependen de esos paquetes de aplicación.

Una instalación que respete estas dependencias de ejecución incorpora esa cadena. No se afirma que todo comando `colcon build` instale automáticamente paquetes ni que toda compilación aislada falle: resolver dependencias, seleccionar paquetes, construir e iniciar nodos son operaciones distintas. setup.py de core tiene install_requires=setuptools; pip no replica por sí solo las dependencias de package.xml.

El mismo bringup contiene tanto launch Pi como PC y declara RViz/SLAM junto con los paquetes de ambas máquinas. Un flag launch puede evitar iniciar un nodo, pero no elimina la dependencia declarada por el paquete. Este es el desacople principal a resolver en un futuro perfil robot/PC.

## D02 · Confirmado y precisado · El dashboard busca un recurso del orquestador

`src/rescue_command_station/rescue_command_station/nodes/dashboard_node.py:959–973` busca slam_rviz.rviz dentro de rescue_bringup y, si no encuentra ese paquete, intenta `/workspace/src/rescue_bringup/config/slam_rviz.rviz`.

Es una dependencia de una acción de UI (abrir RViz), no un import que impida siempre arrancar el dashboard. El fallback puede funcionar si existe ese checkout; el código no garantiza que exista ni que corresponda a la versión instalada. Tampoco se puede garantizar un fallo total de RViz solo por no encontrar su configuración: esa ejecución no se hizo.

No solucionar simplemente declarando station→bringup: bringup ya depende de station. Opciones futuras: situar la configuración RViz en un paquete de recursos sin dependencias hacia las aplicaciones, o inyectar la ruta desde el launch PC. La estación no debería localizar archivos dentro de su orquestador.

## D03 · Confirmado y ampliado · Core es requisito del brazo PC también en modo real

`src/rescue_command_station/launch/arm_station.launch.py:28–36` resuelve servos.yaml de core antes de las condiciones `sim`. Por tanto `sim:=false` sigue necesitando core en el índice ament del PC, aunque los drivers reales se ejecuten en la Pi.

En `sim:=true`, `:54–62` arranca dynamixel_sim_node desde core. Ese simulador importa leer_servos_params y WheelEncoder de core (`nodes/dynamixel_sim_node.py:33–34`), pero no GPIO, pyserial ni Dynamixel SDK. Core agrupa simulador, utilidades puras y drivers reales en un solo paquete con dependencias de hardware.

Mover solo el archivo del simulador no elimina sus imports de core; habría que ubicar también las utilidades puras que realmente necesita en una dependencia neutral, o definir una implementación del simulador independiente del driver físico. No duplicarlas silenciosamente.

El simulador matemático de escritorio `arm/simulator.py` es otra cosa: configured_arm/settings usan geometría de station y no llaman joint_drivers. No necesita por su lógica el SDK ni un robot conectado; sí necesita sus dependencias de UI/matemáticas y acceso a arm.yaml. No confundirlo con la simulación ROS que levanta dos buses.

## D04 · Confirmado · Dependencias omitidas no son necesariamente cruces PC/robot

Bringup y core usan ament_index_python sin declararlo directamente; station sí lo declara. Algunas funciones importan pero no usan el símbolo, por ejemplo camera.launch.py; eso sigue haciendo que el módulo necesite poder importarse. Puede llegar transitivamente desde otras dependencias: no se ha reproducido un import fallido en una instalación mínima ROS.

Este es un defecto de declaración/reproducibilidad que debe revisarse, pero ament_index_python es infraestructura compartida, no código específico del PC o del robot. Corregir la lista de dependencias no equivale a desacoplar los dominios.

## D05 · Rectificado el recuento · Seis literales ejecutables, más tres ejemplos

Entre los nueve sitios enumerados en la versión anterior había tres ejemplos dentro de docstrings: vision.launch.py:20,25 y logitech_vision.launch.py:12. No son nueve defaults que se ejecutan.

Los seis sitios ejecutables de esa lista son:

- bringup/launch/slam.launch.py:237.
- bringup/launch/vision.launch.py:106 y :137.
- bringup/launch/logitech_vision.launch.py:63.
- bringup/rescue_bringup/object_detector.py:211.
- command_station/rescue_command_station/nodes/dashboard_node.py:963.

Las rutas a modelos dentro de `/workspace/src/rescue_bringup` acoplan el runtime al checkout montado en el contenedor. El setup de bringup ya instala modelos en share; las rutas ejecutables revisadas no aprovechan esa ubicación. Este es acoplamiento al despliegue, no necesariamente dependencia de una máquina remota. Hay otras rutas en scripts; seis no pretende ser el total del repositorio.

## D06 · Confirmado y ampliado · Configuración y topología física filtradas hacia PC

`arm/configuration.py:45–69` lee servos.yaml de core, reconoce los nodos ax12a_driver/ex106_driver, extrae pares bus/ID y verifica que sus articulaciones coincidan con joint_order. `cartesian_node.py:22,31–32,227` usa esa distribución para dividir comandos entre buses. GUI (`gui_node.py:231–250,360–361`) selecciona servicios AX/EX e IDs de servos para mantenimiento.

Existe un acoplamiento semántico a la topología física incluso sin importar el driver. Cambiar el hombro a otro bus puede exigir cambios de configuración/código del PC. Un esquema compartido de nombres y capacidades podría justificar parte de esa información; las rutas de puertos, baudios, GPIO y detalles internos del driver deberían permanecer del lado robot.

Además, `configuration.py:46` resuelve el paquete por defecto **antes** de leer servos_config_path en :48. arm_station hace lo mismo antes de declarar servos_config. Pasar una ruta personalizada no evita por sí solo la resolución inicial de core. Orden confirmado además con una reproducción aislada de joint_drivers y un resolver falso, sin ejecutar launch: [dependencias_config_reproduccion.txt](dependencias_config_reproduccion.txt). El override no llega a consultarse si primero falla la resolución de core.

No es correcto concluir que arm.yaml y servos.yaml deban fundirse en uno: geometría y calibración son dominios distintos. Tampoco basta copiar servos.yaml a station: serían dos archivos editables de la misma configuración. Una configuración compartida debe tener una fuente mantenida y versiones coherentes; cada despliegue puede recibir su copia derivada. Alternativamente, el robot puede exponer la descripción necesaria mediante un contrato de capacidades, sin transferir su configuración privada completa.

## D07 · Confirmado y reclasificado · Sustitutos locales de bibliotecas

cv_bridge local y del sistema tienen el mismo nombre, con APIs distintas (H08). Es selección de paquete/import por entorno, no enlazado de un driver PC con Pi. Lo mismo ocurre con joy. No demuestra un cruce entre las dos aplicaciones.

El joy que usa el perfil command_station se ejecuta donde está el mando, normalmente PC; no debe etiquetarse como dependencia obligatoria compartida con el robot solo porque esté en src/dependencias. cv_bridge se necesita en los procesos que realmente lo importan, no automáticamente en ambas máquinas. El inventario de proveedores debe distinguir rol de despliegue de nombre de carpeta.

## D08 · Rectificación importante · Dockerfile PC sí tiene automatización

La afirmación anterior «Dockerfile PC huérfano y ambos scripts siempre construyen Pi» era incorrecta:

- run_pi_sensors.sh:60 construye explícitamente Dockerfile.pi.
- run_slam_container.sh:98–100 construye el contexto con su Dockerfile predeterminado, el Dockerfile de raíz, si falta imagen o se pide rebuild.
- Solo la rama rebuild-pi (:92–96) de este último selecciona Dockerfile.pi expresamente.
- La rama arm (:183–194) instala requirements_pc y Chromium, construye hasta station y lanza el brazo. Sí existe una ruta automatizada para la estación.

El problema confirmado es reutilizar el tag `pedros-rescue-ros2:jazzy` para recetas PC y Pi dentro del flujo local, sin validar qué receta produjo una imagen existente. En la misma máquina/almacén puede reutilizarse una imagen del rol equivocado después de rebuild-pi. PC y Raspberry normalmente tienen almacenes de imágenes distintos: el mismo nombre por sí solo no hace que compartan una imagen entre máquinas ni prueba que el PC ejecute siempre la receta Pi. No se inspeccionó Podman ni la imagen realmente instalada.

## D09 · Nuevo · Omitir paquetes del build no crea una frontera de instalación

run_pi_sensors.sh:47–51 usa rosdep sobre todo src con ignore-src y después excluye rescue_command_station de colcon. El manifiesto de bringup sigue declarándola como dependencia. Con todos los paquetes presentes en src, rosdep puede tratar los locales como responsabilidad del workspace; omitir luego su construcción no vuelve válido un manifiesto que los exige. No se ha probado qué error concreto produce este entorno ni se afirma que rosdep descargue la estación desde un registro.

La receta PC hace un build amplio (:114–115) y la rama arm construye hasta station (:191), que declara core. Las exclusiones/listas en scripts son parches de selección; falta una definición verificable de las dependencias de cada perfil.

## D10 · Nuevo · La independencia operativa necesita un contrato de desconexión

El robot no debería depender de que una GUI siga viva para mantener sus límites y ejecutar una parada válida. Actualmente cartesian_node y varias validaciones viven en PC; los drivers de Pi tienen protecciones propias pero con las limitaciones H05/H23. Desacoplar carpetas o mover YAML no corrige esos fallos.

Mantener planificación/GUI en PC puede ser una decisión válida. El robot debe decidir localmente si acepta una orden, su vigencia y qué hace al perder cliente o feedback. No se recomienda trasladar toda la cinemática a la Pi como condición del desacople: eso es una decisión separada de cómputo/latencia. Primero fijar responsabilidades y semántica de comandos, cancelación y emergencia.

## Arquitectura recomendada — propuesta, no cambio aplicado

Conservar el repositorio y crear fronteras por responsabilidad:

| Capa | Contenido | Dependencia permitida |
|---|---|---|
| Contratos compartidos | rescue_interfaces: mensajes/servicios, unidades y capacidades | Tipos ROS básicos; sin importar station/core. |
| Modelo/configuración compartida, si ambas partes la necesitan | Descripción física lógica, articulaciones, datos versionados; utilidades matemáticas puras | Contratos o bibliotecas puras; sin GUI, drivers ni launch de ambas aplicaciones. |
| Robot | Motores, buses, sensores locales, validación/parada del hardware, configuración privada | Contratos y recursos realmente compartidos. Sin station/Tk/Chromium. |
| PC | Dashboard, mando, herramientas de misión, planificación y visualización elegidas para PC | Contratos y recursos necesarios. Sin drivers reales/core. |
| Simulación opcional | Proveedor falso del contrato del robot y recursos de prueba | Contratos/utilidades puras. No requerir instalar el paquete de drivers reales. Puede vivir inicialmente junto a herramientas PC si es exclusivamente suyo. |
| Arranque PC y arranque robot | Composición propia de cada perfil | Cada uno depende solo de sus componentes y recursos compartidos. Si se exige independencia de instalación ROS, separar los paquetes de bringup; dos launch en el mismo paquete con todas las dependencias no bastan. |

Percepción/SLAM/exportadores son capacidades, no sinónimo de «hardware Pi». Se instalan donde se haya decidido ejecutarlas. Sacarlas del paquete que también obliga a instalar ambas aplicaciones evita que usar un sensor arrastre una GUI. No crear un nuevo paquete por cada archivo: extraer únicamente responsabilidades que necesiten desplegarse o probarse de manera distinta.

La GUI de mantenimiento puede mostrar buses o IDs cuando eso ayuda al operador a diagnosticar hardware; no es necesario prohibir esa información. Lo que conviene evitar es que los comandos normales dependan de una copia privada del mapa de buses en el PC. El robot podría resolver articulación→driver y publicar capacidades; el mantenimiento especializado puede usar un contrato separado y explícito.

Primer desacople concreto propuesto: recursos RViz/configuración sin depender del orquestador; separar simulación de drivers reales con sus utilidades puras; resolver rutas explícitas antes de exigir defaults; perfiles bringup separados. Después revisar si el enrutamiento por bus debe quedar íntegramente dentro del robot. En ningún caso retirar dependencias del XML antes de eliminar sus consumidores reales.

## Criterios de aceptación de una futura implementación

1. Instalar y abrir estación sin rescue_robot_core ni SDK/GPIO; ausencia de robot se muestra como desconexión, sin error de búsqueda de paquete.
2. Arrancar drivers/sensores del robot sin station, Tkinter, Chromium o RViz; con una política local comprobada de comandos caducados.
3. Ejecutar simulación ROS con el mismo contrato y sin acceso a /dev/USB/GPIO ni dependencia de drivers reales. Probar simulador matemático por separado.
4. Preparar cada perfil desde un workspace limpio con sus dependencias transitivas; no apoyarse en overlays ya presentes ni en la instalación de la otra aplicación.
5. Mismo contrato de articulaciones/unidades y configuración compatible: no duplicar YAML mantenidos a mano ni sobrescribir solo parte del despliegue remoto.
6. Cambiar una implementación de driver conservando el contrato público sin modificar GUI/controladores del PC, salvo herramientas de mantenimiento explícitamente específicas.
7. Configuración RViz/modelos accesible desde paquetes instalados fuera de /workspace; imágenes de PC/robot identificables por receta y versión.
8. Probar pérdida/reconexión de PC, robot y joystick y la respuesta de cancelación; los tests de paquetes por separado no prueban esto.

## Resumen y estado

D01–D03/D06 son acoplamientos de aplicación/recursos confirmados; D04 es declaración de infraestructura; D05 portabilidad; D07 selección de biblioteca; D08 mezcla potencial de recetas, con la afirmación de Dockerfile huérfano retirada. D09/D10 amplían instalación y operación. H01–H29 conservan sus IDs y no se marcan resueltos por esta revisión.

Completado: revisión de D01–D08, extracción estática, rectificaciones, hallazgos D09–D10 y propuesta de independencia. Pendiente: aplicar un desacople si el usuario lo solicita, construir perfiles limpios y validar grafo/desconexiones. Se reprodujo únicamente el orden de resolución de configuración con función AST y resolver falso. No se ejecutaron rosdep/colcon, contenedores, nodos, simuladores ni hardware durante esta ampliación.
