# Brazo 6R: integración del código entregado

Integración local del 10 de septiembre de 2026, en `feat/cinematica-6r-unificada`.
La rama se creó desde `todo-organizado` (`59fbf0d`). Se fusionó el contenido de
`test/cinematica` (`5c07b57`), conservando los cambios de ambos lados; no se creó
un commit ni se publicó una rama. El árbol inicial estaba limpio. La rama
`test/cinematica` sigue conservando la implementación anterior y su historial.

Los archivos del ZIP se comprobaron contra su manifiesto antes de instalarlos.
`kinematics.py`, `motion.py` y `cartesian_controller.py` son idénticos a la entrega:
no se ha escrito otro solver, Jacobiano, interpolador ni planificador.
La sustitución del solver anterior es parte explícita de las instrucciones;
la configuración geométrica local se conservó, como se detalla abajo.
La procedencia y las decisiones de fusión están en
[docs/integracion_6r](docs/integracion_6r/VALIDACION.md).
La auditoría posterior y sus pendientes están en [AUDITORIA_BRAZO.md](docs/integracion_6r/AUDITORIA_BRAZO.md).

## Qué vista se integra

El panel derecho de la GUI existente sustituye los tres gráficos Matplotlib
por el **Canvas del simulador original**, incluyendo cámara blanca, barra amarilla,
aros y ejes locales, cuadrícula, rotación, zoom y vistas frontal/superior.
La cámara frontal, QR, dashboard, controles de emergencia, home, calibración y
rescate siguen en sus paneles existentes.

El dibujo procede de `simulador_web_original/dist/app.js` y se extrae con
`python src/rescue_command_station/tools/extract_arm_canvas.py`.
La extracción conserva las funciones de dibujo y gestos; sustituye la consulta
FK por los datos recibidos de Python y añade una superposición de preview.
`web_view.py` serializa los resultados de la FK suministrada (incluidos los
ejes, orígenes, flange, herramienta y cámara), sin derivarlos de nuevo.

Tk no ejecuta Canvas de HTML. `ArmCanvas` usa Chromium sin ventana, mediante
Playwright, en un hilo separado, y presenta sus fotogramas PNG en el panel Tk.
Los gestos de vista se envían al mismo renderer. El cálculo, la captura y el
browser no bloquean el hilo de callbacks ROS; Tk solo recibe el último fotograma.
La frecuencia objetivo está limitada a menos de 10 capturas/s y depende del PC.
No es un WebView nativo: el transporte es Python → Canvas → PNG → Tk.
No se carga `kinematics.js`, `motion.js` ni un worker de planificación en esta vista.
No hay servidor HTTP, órdenes de hardware ni fallback a un solver JavaScript.
Si falta Chromium aparece un error explícito en el panel.

Las tres interfaces tienen alcances distintos:

| Interfaz | Modelo | Uso |
| --- | --- | --- |
| GUI ROS, panel Canvas integrado | Python compartido | Feedback real/simulado y preview; controles de la estación |
| `arm.simulator` entregado | Python compartido | Laboratorio de escritorio independiente, sin publicadores; su diseño Matplotlib es distinto |
| `simulador_web_original/dist` íntegro | JavaScript original | Demostración autónoma con todos sus controles; no se conecta a ROS |

No se ha trasladado toda la interfaz web de planificación a Tk. La sustitución
solicitada afecta a la **representación gráfica del brazo**; FK/IK y movimientos
en la estación usan los controles existentes y el motor Python entregado.

## Preparar y abrir sin ROS ni robot

Desde la raíz del repositorio, en PowerShell:

```powershell
python -m venv .venv-arm
.venv-arm\Scripts\python -m pip install numpy PyYAML matplotlib playwright customtkinter
.venv-arm\Scripts\python -m playwright install chromium
$env:PYTHONPATH = "$PWD\src\rescue_command_station"
.venv-arm\Scripts\python -m rescue_command_station.arm.web_view
# Laboratorio de escritorio suministrado, con sus controles de simulación:
.venv-arm\Scripts\python -m rescue_command_station.arm.simulator
```

`arm.web_view` es una ventana de comprobación del Canvas en una postura fija:
permite inspeccionar la vista, no controla articulaciones ni hardware.
En `arm.simulator`: copiar la pose, resolver IK, preparar desplazamientos en
`base`/`tool`/`camera`, planificar, reproducir y exportar CSV. Los cambios de
geometría en esa ventana son temporales y no escriben `arm.yaml`.

Para ver el web íntegro, desde la raíz:

```powershell
python -m http.server 8000 --bind 127.0.0.1 --directory src/rescue_command_station/simulador_web_original/dist
```

Abrir `http://127.0.0.1:8000`. HTTP evita las restricciones de workers de `file://`
en la demostración. La vista integrada no necesita esos workers.

## Arranque ROS 2 en Linux (pendiente de ejecutar)

Usar el mismo Python que ejecutará los nodos. Instalar `requirements_pc.txt`
según el procedimiento habitual del repositorio y descargar Chromium:

```bash
python3 -m playwright install --with-deps chromium
# Con ROS 2 y sus dependencias ya instalados y sourced:
colcon build --symlink-install --packages-up-to rescue_command_station
source install/setup.bash
ros2 launch rescue_command_station arm_station.launch.py sim:=true
```

El paquete instala `arm/web_assets/index.html` y `canvas.js` como recursos Python.
También expone `ros2 run rescue_command_station arm_canvas_preview` y
`ros2 run rescue_command_station arm_simulator`. El navegador se instala aparte;
`rosdep`/`colcon` por sí solos no descargan Chromium.
El extra pip `rescue_command_station[canvas]` declara Playwright.

Los nodos de drivers tienen ahora un heartbeat de **status** a 2 Hz, necesario
para la supervisión de la estación. Debe desplegarse también esa actualización
en el entorno simulado y en la Pi antes de probar la versión integrada.
En los drivers **reales**, solo cambió la publicación del estado; los contadores,
protocolos, calibración, regulación y bucles de control no se modificaron.
El driver simulado además publica ángulos con signo, cierra su hilo y expone
telemetría sintética/limpieza de alertas, identificadas como simuladas.

## Fuente de verdad y convenciones

| Dato o cálculo | Fuente |
| --- | --- |
| Geometría, TCP activo, límites lógicos y signos modelo/driver | `config/arm.yaml` |
| FK, IK exacta, Jacobiano y marcos relativos | `arm/kinematics.py` |
| Trayectoria y validación geométrica | `arm/motion.py` |
| Lectura común de configuración | `arm/configuration.py` |
| ID, reducción e inversión física del motor | `rescue_robot_core/config/servos.yaml` |

Metros y radianes internamente. Los controles angulares del simulador muestran
grados del **modelo**, no ticks ni grados de eje motor. La convención es:

```
T06 = Rz(q1) Tz(h)
      Ry(-q2) Tz(L1) Ry(-q3) Tz(L2) Ry(-q4) Tz(L3)
      Rz(q5) Tx(e56) Rx(q6)
Ttool = T06 Tx(tool_length)
Tcamera = T06 Trans(camera_xyz) Rz(yaw) Ry(pitch) Rx(roll)
```

Con q=0, los tres eslabones están verticales: J5 gira sobre +Z de la base y
J6 sobre +X. Fuera de esa postura, sus ejes se transportan con las articulaciones
anteriores. Tanto la barra como la cámara giran con J6; la punta sobre su eje
no se traslada, pero una cámara descentrada describe una órbita.

`fk()['flange']` es T06 geométrica; por compatibilidad con las interfaces previas,
`fk()['T06']` devuelve la pose del **TCP activo**, tool o camera. `points` termina
en la punta de la herramienta aunque el TCP activo sea la cámara.

`q_model = model_sign * q_driver`, y la conversión inversa usa los mismos signos.
No se ha cambiado la calibración de los drivers ni su inversión física.
Se conserva el signo lógico anterior de J4, pendiente de comprobación física.

Para un desplazamiento expresado en F:
`p_objetivo = p_TCP + R_base_F * delta_F`, conservando la orientación del TCP.
Las columnas de la cámara se obtienen del montaje real, no se asume que coincidan
con las de la herramienta. La orientación fija del montaje debe calibrarse.

## Inversa y trayectorias

La IK retira el montaje del TCP y el desplazamiento axial J5–J6, resuelve las
ramas de base, descompone `Ry(-beta) Rz(q5) Rx(q6)` y resuelve el plano 2R
después de retirar L3. `beta=q2+q3+q4`. Se ajustan vueltas completas dentro de
los límites, se verifica **posición y orientación** mediante FK y se elige la
solución más cercana a la semilla. Las familias singulares se muestrean; no se
promete encontrar todas sus soluciones.

`/compute_ik_pose` es la única interfaz IK de la estación. Se retiraron
`ComputeIK.srv`, `/compute_ik`, el visor `simulacion_node.py`/`arm_sim_node`
y los canales de puntos/preview heredados. La GUI introduce XYZ y Roll/Pitch/Yaw
(grados; R = Rz(yaw) Ry(pitch) Rx(roll)). Copiar una pose conserva su orientación.
«Resolver IK: solo preview» nunca envía motores. Todos los botones cartesianos,
incluidos los incrementos XYZ, pasan por el planificador completo.
El método inactivo `component_target` permanece dentro del **archivo original
del ZIP**, sin consumidores en la estación, para conservar íntegro el motor.
No es el solver anterior ni una segunda ruta de control.

Los segmentos usan `s(u)=10u³−15u⁴+6u⁵`; posición cartesiana lineal y orientación
SLERP, incluida la rotación de 180°. La semilla IK de cada muestra es la anterior.
Se refinan intervalos con saltos mayores de 2° o desviaciones de la interpolación
articular superiores a 0.05 mm / 0.02° en sus puntos de cuarto. Son comprobaciones
discretas, no una cota analítica continua. Los waypoints se recorren con parada
en cada unión. En `/cartesian/goto`, `step_dt` es el intervalo base y el tiempo total es
`max(0.2, n_steps * max(0.02, step_dt))`; `step_dt <= 0` usa 0.05 s.
En `/cartesian/trajectory`, cada `CartesianWaypoint.duration` es el **total del
segmento**, conforme al mensaje y a la GUI existente. Se acepta entre 0.2 y 120 s.
Cada duración se adapta a `plan_waypoints` sin modificar el planificador entregado;
distintos segmentos pueden tener duraciones distintas. El comentario del mensaje
se actualizó al rango real del planificador.
`n_steps` es una resolución solicitada: el planificador puede densificarla.
El simulador también usa segundos totales por segmento.

El controlador valida todo el recorrido antes de publicar. No devuelve ni ejecuta
planes parciales y no recorta ángulos después de IK. Las peticiones durante una
trayectoria activa se rechazan. Los botones de teleoperación usan ese planificador.
Los comandos articulares directos de la GUI se envían completos a
`/arm/manual_joint_cmd`; `cartesian_node` valida los seis nombres, valores,
límites, feedback y buses, y es quien publica hacia los dos drivers.
Se rechazan durante **planificación y ejecución**, bajo la misma exclusión de
admisión que las trayectorias. No se cambian los protocolos de los drivers.
El arbitraje cubre las órdenes articulares de la estación; no puede impedir
que un cliente externo publique directamente en topics o servicios de drivers.

Se requieren estados recientes de las seis articulaciones y de ambos buses.
`/cartesian/speed_scale=0` pausa el envío. `/cartesian/cancel` cancela futuros
puntos; **no es un paro de emergencia**: el servo puede terminar su último objetivo.
El feedback usa FK de las articulaciones medidas, no errores de IK ficticios.
`success` en el servicio significa plan aceptado; la ejecución es asíncrona.
La GUI distingue finalización de detención/cancelación; no marca 100% al cancelar.
El estado `in_progress` incluye preflight. La recepción de feedback sigue en
callbacks separados con un ejecutor de tres hilos. No se publica una FK medida
hasta tener las seis articulaciones recientes. El Canvas indica explícitamente
cuando muestra una última pose o un cero de referencia sin feedback completo.
El cliente libera su petición pendiente al responder; tras 60 s sin respuesta
solicita cancelación y comunica el error, sin reintento silencioso. El servidor
conserva la exclusión hasta que termina el cálculo que estaba en curso.

Se publican `/end_effector_pose` (TCP activo), `/tool_pose` y `/camera_pose`.
El visor recibe un snapshot local de la misma FK, sin canales de puntos antiguos.
`CartesianState` informa `maintenance` y `feedback_valid`; XYZ es NaN sin feedback
reciente. Los consumidores deben regenerar las interfaces ROS de esta versión.

El lanzamiento comparte `arm_config` y `servos_config` explícitos entre nodos.
En ROS, el cargador usa el overlay instalado; las pruebas offline usan el checkout.
Los parámetros geométricos se declaran de solo lectura al iniciar. Se rechazan
límites invertidos, nombres repetidos, IDs que colisionan y discrepancias entre
las seis articulaciones y los `servo_names` activos. Cambiar archivos requiere
reiniciar el conjunto con el mismo perfil; no mezclar overrides por nodo.

La GUI se muestra por defecto al arrancar sola. El dashboard la lanza con
`managed_by_dashboard:=true`; `gui:=false` permite comprobar solo nodos.
Al cerrar la GUI independiente termina su lanzamiento, sin señales al grupo
de procesos de la terminal. Debe existir **un solo arm_station** en el dominio:
no iniciar otro si el dashboard ya lo precargó.
En modo independiente la GUI publica `/arm_active` para inhibir el teleop de
la base, y lo libera al cerrar normalmente. El dashboard conserva esa función
cuando gestiona la ventana.

## Mantenimiento y rescate

«Iniciar calibración» adquiere primero `/cartesian/maintenance` (`SetBool=true`).
El controlador cancela nuevos puntos y bloquea admisión/publicación normales
bajo el mismo cerrojo. La GUI solo habilita jog/rescate cuando ambos buses
confirman calibración, están recientes y sin emergencia, y el ejecutor está
inactivo. Jog además exige ángulo reciente; rescate no requiere encoder fresco.
«Confirmar home» libera el bloqueo **después de dos respuestas exitosas** de
calibración. Un fallo parcial conserva el bloqueo; repetir inicio y confirmación.
No usar confirmación de cero como un botón de reanudación: modifica la referencia
de los encoders, como en los drivers originales.

El bloqueo no caduca por pérdida de la GUI. Si un servicio no responde, mantener
los movimientos normales bloqueados y revisar los nodos antes de reiniciar la
estación; no se recupera ni se libera automáticamente tras una respuesta incierta.
No es un bloqueo distribuido persistente y no sobrevive al reinicio del controlador.
Los servicios directos de drivers/publicadores externos siguen fuera de esta
exclusión; ver los hallazgos abiertos de la auditoría.

## Antes de conectar el brazo real

La configuración actual conserva los valores encontrados en la rama local.
Esto es una desviación **deliberada y documentada** de los defaults de la entrega,
para cumplir la petición de conservar los cambios locales sin inferir medidas.

| Parámetro (m) | Local conservado | Entrega Python |
| --- | ---: | ---: |
| base_height | 0.12 | 0.14 |
| L1 | 0.36 | 0.30 |
| L2 | 0.36 | 0.30 |
| L3 | 0.00 | 0.10 |
| tool_length | 0.00 | 0.10 |

**Con L3=0 y tool_length=0 no aparece un tercer eslabón de longitud visible ni
una barra amarilla extendida.** No es un cambio del renderer. Para reproducir
las dimensiones de la entrega, revisarlas y editarlas explícitamente en
`config/arm.yaml`; no se han aplicado a la configuración física automáticamente.
La captura `canvas_entrega.png` de la verificación usa .30/.30/.10 y herramienta
.10 solo en memoria; `canvas_config_local.png` usa los valores locales.

- Verificar mediante medición qué perfil geométrico corresponde al brazo.
- e56=0 y camera_xyz=(0.06,0,0.04) con RPY=0 son **provisionales**.
- Comprobar cero vertical, signos J1–J6, distancia J5–J6, TCP y calibración hand-eye.
- Confirmar límites mecánicos, velocidad y aceleración admisibles. Los límites
  lógicos se conservaron del software anterior, no son una certificación mecánica.
- Empezar en simulación, después verificar un eje a la vez a baja velocidad, con
  el paro del driver disponible y el área despejada.

No se implementan colisiones, gravedad, carga, par, límites dinámicos ni control
de fuerza. El perfil cartesiano no garantiza que los servos sigan los tiempos
planeados. El CSV contiene radianes del modelo y no debe enviarse directamente
a motores. Ninguna prueba de desarrollo acciona hardware.

## Pruebas reproducibles

```bash
PYTHONPATH=src/rescue_command_station python -m unittest discover -s src/rescue_command_station/test -v
python -m compileall -q src/rescue_command_station/rescue_command_station
```

En PowerShell configure primero PYTHONPATH como en el ejemplo inicial.
Incluyen 1.000 poses aleatorias contra productos homogéneos independientes,
FK/IK, Jacobiano, singularidades, órbita de cámara, marcos, trayectorias densas,
límites, entradas inválidas y lógica de ejecución con transporte simulado.
Estas pruebas sin ROS no sustituyen `colcon build`, una prueba del grafo ROS,
la inspección visual de Tk ni la validación física.

## Pendientes explícitos

- **No se ejecutó ROS 2 ni `colcon build`.** `setup.py build_py` solo verificó
  el empaquetado Python y sus assets; no demuestra un grafo ROS funcional.
- En `sim:=true`, comprobar que ambos publicadores aportan los seis nombres a
  `/arm/joint_states`, que `/ax12a/status` y `/ex106/status` llegan a 2 Hz y que
  una pausa de feedback >1 s o status >2 s detiene futuros puntos.
- Probar preflight largo, pausa por velocidad cero, cancelación durante preflight
  y ejecución, desconexión, emergencia, reconexión y pérdida de un solo bus.
  Los tests con transporte falso cubren lógica, no descubrimiento DDS/QoS.
- Probar en la pantalla real la GUI completa, scroll, cámara/QR, dashboard,
  todos los controles, joystick y estabilización. Se comprobó el widget Canvas
  y la construcción de todas las pestañas de la GUI con transporte falso y
  ventana oculta; no la GUI conectada a ROS.
- El renderer usa capturas PNG, no composición nativa de WebView: medir consumo,
  latencia y fluidez en el PC de destino. El input de vista puede tardar un
  fotograma; la red y la planificación ROS no dependen de ese renderer.
- El mantenimiento iniciado desde la GUI adquiere exclusión central; verificar
  también fallos parciales, pérdida de GUI y reinicios en ROS. Los clientes
  externos que llaman directamente a drivers pueden saltarse esa exclusión.
  Los `rescue_pulse` reales heredados no comprueban emergencia al entrar:
  hallazgo abierto de bajo nivel, descrito en la auditoría; no se modificaron.
  Tampoco se promete un paro físico por cancelar futuros puntos.
- La demostración web ofrece IK solo de posición; esa opción no se trasladó al
  motor Python, que resuelve poses completas. No hay paridad de esa opción.
- La **posición adicional persistente** descrita al final del Markdown original
  no se implementó: no forma parte de sustituir la vista. No se redefine home,
  no se guarda una pose nueva ni se mueve el robot al arrancar.
- No se midieron geometría, signos, offsets ni calibración; no hubo hardware.
  No hay validación de colisiones, dinámica, carga, par o seguridad física.

## Verificación específica del Canvas

```powershell
$env:PYTHONPATH = "$PWD\src\rescue_command_station"
python src/rescue_command_station/tools/verify_arm_canvas.py --tk
# Opcional: --output-dir RUTA para conservar capturas
```

Requiere Playwright/Chromium y Tk. Prueba ambos TCP, preview, cambio de J6,
redimensionado, gestos/vistas, ausencia de motor JS en la vista integrada,
recepción del PNG en Tk y cierre del worker. El web íntegro se verifica aparte:

```powershell
Push-Location src/rescue_command_station/simulador_web_original
node verify.cjs
node verify-motion.cjs
Pop-Location
```

Resultados de esta integración y pasos ROS pendientes:
[VALIDACION.md](docs/integracion_6r/VALIDACION.md).
