# Integrar el código 6R adjunto: NO reimplementar la cinemática

## Petición del usuario

Trabaja en `C:\Users\juanu\Documents\GitHub\Pedros-Rescue`, partiendo de
`todo-organizado` y usando una rama nueva `feat/cinematica-6r-unificada`.
Conserva cualquier modificación local. Este paquete contiene código real ya
implementado, no solamente una especificación para volver a programarlo.

**Copia y adapta los módulos suministrados. No vuelvas a derivar ni a escribir
otro solver FK/IK, Jacobiano o planificador. No rediseñes el simulador web.**

No modificar contadores, protocolos ni control de bajo nivel de AX-12A/EX-106+.
No accionar hardware durante la integración. No afirmar que las pruebas
matemáticas certifican el funcionamiento del brazo físico.

## Qué contiene `ENTREGA_CODIGO_6R.zip`

- `simulador_web_original/`: código exacto del simulador web existente, con su
  vista, cámara, herramienta, controles, FK/IK, marcos locales y trayectorias.
  No se ha reconstruido para esta entrega.
- `integracion_python/`: archivos nuevos y modificados de la estación, conservando
  su ruta relativa dentro de `Pedros-Rescue`. Es el código de integración local
  preparado a partir de `todo-organizado`, no un proyecto completo independiente.
- `base_original/`: versiones originales de los archivos modificados que ya
  existían en esa rama. Sirven para una fusión de tres vías sin perder trabajo.
- `MANIFIESTO.json`: commit base, archivos incluidos y huellas SHA-256.
- Este Markdown.

Base exacta utilizada: `59fbf0dc518a98cfea8772b87242ec276084d9cf`.
No se ha publicado ninguna rama remota ni cambiado la carpeta Windows del usuario.

## Abrir el simulador que el usuario ya conoce

En la carpeta `simulador_web_original`, ejecutar:

```powershell
python -m http.server 8000 --bind 127.0.0.1 --directory dist
```

Abrir `http://127.0.0.1:8000`. Se usa HTTP local para evitar restricciones de
workers al abrir directamente con `file://`. El servidor solo expone esa carpeta.

Archivos importantes:

| Archivo | Responsabilidad |
| --- | --- |
| `dist/index.html` | Estructura y controles de la vista original |
| `dist/style.css`, `dist/motion.css` | Apariencia original |
| `dist/app.js` | Interfaz y representación 3D en Canvas |
| `dist/kinematics.js` | FK, IK, Jacobiano implícito y montaje cámara/herramienta |
| `dist/motion.js` | Trayectorias y movimientos relativos |
| `dist/motion-ui.js` | Animación, controles, curvas y exportación |
| `dist/motion-worker.js` | Cálculo de planificación fuera del hilo de interfaz |
| `dist/modelo.html` | Explicación matemática y limitaciones |
| `verify.cjs`, `verify-motion.cjs` | Pruebas del código web |

Ejecutar `node verify.cjs` y `node verify-motion.cjs` desde esa carpeta.

## Motor único dentro del sistema ROS

**Para el sistema integrado, la fuente de verdad ejecutable es Python.**
La traducción del modelo del simulador ya está incluida; no hace falta traducirla
otra vez. El web original se entrega como demostración autónoma y referencia
visual, no como un segundo controlador conectado al robot.

Copiar/fusionar desde `integracion_python` los siguientes módulos:

```text
src/rescue_command_station/rescue_command_station/arm/
    kinematics.py
    configuration.py
    motion.py
    cartesian_controller.py
    cinematica_node.py
    cartesian_node.py
    gui_node.py
    simulator.py
```

Además están incluidos `config/arm.yaml`, `launch/arm_station.launch.py`,
`setup.py`, `package.xml`, `requirements_pc.txt`, `ARM_6R.md` y las pruebas.

La GUI ROS existente y el simulador de escritorio importan el mismo motor Python.
`simulator.py` tiene una **vista de escritorio distinta**, no es una copia visual
idéntica del Canvas web. No afirmar lo contrario al usuario.

Para preservar EXACTAMENTE la vista web dentro de la estación, reutiliza sus
HTML/CSS/Canvas y adapta solamente la comunicación con el backend Python: los
cálculos FK/IK y las trayectorias deben proceder de `kinematics.py`/`motion.py`.
Ese puente web→Python **no está implementado en este paquete**. Si se integra esa
vista, es trabajo de interfaz/transporte, no una reimplementación matemática.
Mantén la demostración JavaScript separada del camino de control real; no añadas
un fallback que silenciosamente resuelva con el otro motor.

## Modelo ya implementado: conservar

Metros y radianes. Cero articular con tres eslabones verticales:

```text
T06 = Rz(q1) Tz(h)
      Ry(-q2) Tz(L1) Ry(-q3) Tz(L2) Ry(-q4) Tz(L3)
      Rz(q5) Tx(e56) Rx(q6)
Ttool = T06 Tx(tool_length)
Tcamera = T06 Trans(camera_xyz) Rz(yaw) Ry(pitch) Rx(roll)
```

L1/L2/L3 = 0.30/0.30/0.10 m. J5 es Z local y J6 X local; la cámara y la barra
giran ambas con J6. Los ejes solo coinciden con Z/X de la base en la postura cero.
No introducir de nuevo el cero horizontal ni el modelo de muñeca anterior.

El TCP controlado (`tool`/`camera`) y los ejes del desplazamiento
(`base`/`tool`/`camera`) son selecciones independientes.

La API heredada `fk()['T06']` devuelve el TCP activo; `fk()['flange']` devuelve
la T06 geométrica. `points` termina en la punta de herramienta. Respeta esas
convenciones al conectar publicadores o renderizadores.

La IK verifica posición Y orientación, respeta límites y usa la semilla para
continuidad. Las familias singulares se muestrean. No sustituirla por una IK
"best effort", ni recortar ángulos después de resolver.

Los planes se validan completos antes de publicar. Perfil temporal quíntico,
SLERP y refinamiento de intervalos están implementados en `motion.py`.

## Procedimiento de integración sin perder cambios

1. Leer AGENTS.md, `git status`, rama y estructura actuales del repositorio.
2. Crear la rama nueva desde `todo-organizado` sin sobrescribir cambios ajenos.
3. Comparar cada archivo del paquete con `base_original` y con la versión local.
   Si el archivo local no cambió respecto de la base, instalar el suministrado;
   si cambió, fusionar las modificaciones conservando lo no relacionado.
4. Usar las clases y funciones adjuntas; adaptar imports, mensajes ROS, eventos
   de GUI y rutas cuando haga falta. No generar otro motor matemático.
5. Retirar las implementaciones cinemáticas anteriores y comprobar todos sus
   consumidores. No eliminar controles de emergencia o calibración.
6. Mantener geometría/límites/signos en `config/arm.yaml`; los mapas de buses/IDs
   se LEEN del archivo de configuración existente de los drivers.
7. Ejecutar las pruebas adjuntas y luego pruebas del grafo ROS simulado.
8. Entregar el diff, comandos de arranque, resultados y pendientes sin afirmar
   pruebas físicas que no se hayan realizado.

## Comprobaciones de integración PENDIENTES conocidas

La matemática está probada; la integración ROS no ha podido arrancarse en el
entorno de origen. Revisar y corregir estos aspectos de integración, sin rehacer
los cálculos:

- `cartesian_node.py` exige estados de buses con antigüedad menor de 2 s. Algunos
  drivers originales publican status solo ante eventos. Verificar esta condición
  para no impedir todos los movimientos pasado ese tiempo. No modificar motores
  o contadores: ajustar la supervisión/heartbeat de estado coherentemente.
- Verificar que los seis estados articulares llegan a `/arm/joint_states`, con
  los remapeos incluidos, y que la planificación no bloquea su recepción.
- Revisar semántica de `duration` y `n_steps` de servicios/GUI: no confundir
  intervalo por muestra con duración total del segmento.
- Asegurar que no compitan órdenes articulares directas y trayectorias; comprobar
  cancelación, desconexión, emergencia y pérdida de feedback con transporte simulado.
- Confirmar que las peticiones del joystick no quedan bloqueadas por flags de
  solicitud pendiente, y que la estabilización tiene una orientación inicial válida.
- Revisar la GUI con pantalla real: disposición, scroll, controles y animación.
- No asumir coincidencia numérica bit a bit entre JavaScript y Python cerca de
  singularidades: pueden seleccionar diferentes soluciones válidas. Comparar poses.
- La versión web tiene IK solo de posición adicional; el port Python incluido
  expone la IK de pose completa. No prometer paridad de esa opción sin implementarla
  explícitamente como tarea separada y probada.

## Configuración y seguridad

`arm.yaml` conserva h=0.14 m y longitud de herramienta=0.10 m del proyecto.
El web autónomo tiene sus propios valores de demostración. **No copiar sus
valores por defecto a la configuración física sin medición.**
El desplazamiento J5–J6 y el montaje de cámara son provisionales. Conservar la
distinción `q_model = model_sign * q_driver`; no modificar la calibración de
bajo nivel por inferencia de un dibujo. Verificar cero vertical y sentidos.

No hay verificación de colisiones ni límites dinámicos/carga/par. Cancelar
detiene el envío de nuevos puntos, pero no sustituye un paro de emergencia del
driver. El simulador no manda órdenes a hardware.

## Pruebas ya ejecutadas en la entrega

- Web: 1.200 FK contra productos homogéneos independientes, 1.200 poses FK/IK,
  100 pruebas IK de posición, singularidades y pruebas de trayectorias.
- Python: 15 tests, con 1.000 poses aleatorias, marcos, Jacobiano, cámara,
  trayectorias y lógica del ejecutor con transporte falso.
- No se ejecutó ROS, `colcon build`, interfaz gráfica Tk ni pruebas físicas.

Después de integrar, desde la raíz del repositorio:

```powershell
$env:PYTHONPATH = "$PWD\src\rescue_command_station"
python -m unittest discover -s src/rescue_command_station/test -v
python -m compileall -q src/rescue_command_station/rescue_command_station
python -m rescue_command_station.arm.simulator
```

En un entorno ROS 2, compilar con `colcon` y probar primero:

```bash
ros2 launch rescue_command_station arm_station.launch.py sim:=true
```

## Posición adicional persistente: NO está implementada todavía

No confundirla con funciones ya existentes en el código adjunto. Si se aborda
durante la integración, añadirla separadamente: guardar seis ángulos con unidades,
convención y huella de configuración; escritura atómica en un archivo del usuario;
carga validada y movimiento solo bajo petición explícita mediante el planificador.
No redefinir home, no mover al arrancar, y rechazar configuraciones incompatibles.
Requiere una referencia de calibración válida para recuperar la pose físicamente.

## Criterio final

El usuario pide integrar SU simulador y SU cinemática ya implementados, no crear
otros parecidos. Mantener el código adjunto como base; cualquier corrección
matemática necesaria debe justificarse con una prueba que falle antes y pase
después. No afirmar que la vista original ya está embebida en ROS: el paquete
incluye el web original y el port de escritorio, y el puente queda por integrar.
