# Arquitectura — Control Brazo 5-DOF ROS2

## Resumen del sistema

El sistema está dividido en **5 nodos ROS2** que se comunican por topics y servicios.
Ningún nodo sabe cómo funciona internamente otro — solo hablan por mensajes.

```
ax12a_driver ──► /joint_states ──► cinematica_node ──► /end_effector_pose ──► gui_node
ex106_driver ──►                                   └──► /fk_points         ──► simulacion_node
                                   /compute_ik  ◄────────────────────────────  gui_node
```

---

## Carpetas

### `config/`
Parámetros externos que los nodos leen al arrancar. **Nada de esto está hardcodeado en el código.**

| Archivo | Qué contiene |
|---------|-------------|
| `servos.yaml` | IDs de servos, reducciones, deadband, puertos serie, baudrates, geometría del brazo, reach de simulación |

> Si agregás un servo nuevo o cambiás un puerto, solo tocás este archivo. No el código.

---

### `launch/`
Scripts para arrancar el sistema. Un solo comando levanta todos los nodos.

| Archivo | Cuándo usarlo |
|---------|--------------|
| `full_system.launch.py` | Con hardware real conectado |
| `sim_only.launch.py` | Sin hardware, solo cinematica + visualización + GUI |

---

### `msg/`
Mensajes ROS2 personalizados (tipos de datos propios).

| Archivo | Campos | Para qué sirve |
|---------|--------|----------------|
| `ArmStatus.msg` | `conectado`, `emergencia`, `modo_calib`, `mensaje` | Publicar el estado global del robot para que la GUI lo muestre |

---

### `srv/`
Servicios ROS2 personalizados (llamadas request/response).

| Archivo | Request | Response | Quién lo usa |
|---------|---------|----------|-------------|
| `RegisterServo.srv` | `id`, `nombre`, `reduccion`, `deadband_deg`, `invertir_giro` | `success`, `mensaje` | Cualquier controlador que quiera agregar un servo al driver en runtime |
| `ServoCommand.srv` | `id`, `target_deg`, `vel_pct` | `success`, `mensaje` | GUI para jog y pulso de rescate |
| `ComputeIK.srv` | `x`, `y`, `z`, `beta_deg`, `q5_deg`, `elbow` | `success`, `q_rad[]`, `mensaje` | GUI para calcular cinemática inversa |

---

### `control_brazo/` — El paquete Python

---

## Archivos de código

---

### `wheel_encoder.py`
**Qué es:** Clase Python pura. No sabe que ROS existe.  
**Problema que resuelve:** Los servos Dynamixel solo ven 300° mecánicos. Con una reductora 27:1 el eje de salida puede dar muchas vueltas, pero el sensor interno rebota entre 0° y 300°. Esta clase detecta esos rebotes y acumula las vueltas reales.

**Responsabilidades:**
- Acumular vueltas completas (wrap-around 0-1023 ticks)
- Filtro de mediana de 3 muestras para suavizar ruido del sensor
- Rechazar lecturas espurias (saltos > 80 ticks, hasta 5 consecutivas)
- Calcular `angulo_salida = (angulo_motor / reduccion) - offset_calibracion`
- Calcular error circular para el controlador de posición

**Usada por:** `ax12a_driver_node.py` y `ex106_driver_node.py` internamente. Nadie más la ve.

**Estado de implementación:**
| Función | Estado |
|---------|--------|
| `update(pos_raw)` | ✅ Completo |
| `set_zero()` | ✅ Completo |
| `reset_full()` | ✅ Completo |
| `error_circular()` | ✅ Completo |

> **No hay pendientes.** Este archivo está completo y listo.

---

### `kinematics.py`
**Qué es:** Biblioteca Python pura de matemáticas. No sabe que ROS existe.  
**Problema que resuelve:** Calcular dónde está el efector final dado un set de ángulos (FK), y calcular qué ángulos necesito para llegar a una posición cartesiana (IK).

**Responsabilidades:**
- Transformaciones homogéneas: `rotz`, `roty`, `transl`, `make_T`
- Extraer Roll/Pitch/Yaw de una matriz de rotación: `rot_to_rpy`
- Cinemática directa `Arm5DOF.fk()`: devuelve la pose del efector y posición de cada articulación
- Cinemática inversa `Arm5DOF.ik()`: dado un objetivo cartesiano, devuelve q[1..5]
- `ArmParams`: dataclass con las dimensiones físicas del brazo

**Usada por:** `cinematica_node.py` únicamente. Los otros nodos reciben los resultados ya calculados via topics.

**Estado de implementación:**
| Función | Estado |
|---------|--------|
| `rotz`, `roty`, `transl`, `make_T` | ✅ Completo |
| `rot_to_rpy` | ✅ Completo |
| `Arm5DOF.fk()` | ✅ Completo |
| `Arm5DOF.ik()` | ✅ Completo |

> **No hay pendientes.** Este archivo está completo y listo.

---

### `ax12a_driver_node.py`
**Qué es:** Nodo ROS2. Maneja el puerto U2D2 y todos los servos AX-12A del robot.  
**Importante:** Un solo nodo para **todos** los AX-12A del robot entero (brazo, gripper, etc.), porque todos comparten el mismo U2D2 físico.

**Responsabilidades:**
- Abrir puerto serie U2D2 al recibir `/ax12a/connect`
- Registrar servos (desde YAML al arrancar, o via servicio en runtime)
- Loop de lectura/control a ~66Hz por cada servo registrado
- Publicar posición actual de todos los servos en `/joint_states`
- Recibir comandos de movimiento en `/ax12a/joint_cmd` (bloqueado en modo calibración)
- Control de posición con velocidad trapezoidal (usando WheelEncoder)
- Respetar `velocity[]` del mensaje `joint_cmd` como `vel_pct` por joint

**Diseño de calibración:**
- `_cb_joint_cmd` bloquea cuando `modo_calib=True` (evita movimientos accidentales)
- El loop de control ejecuta `target_deg` independientemente del modo (permite jogs)
- `_srv_jog` setea `target_deg` directamente sin pasar por `_cb_joint_cmd`

**Topics:**
| Topic | Tipo | Rol |
|-------|------|-----|
| `/joint_states` | `sensor_msgs/JointState` | Publica |
| `/ax12a/joint_cmd` | `sensor_msgs/JointState` | Suscribe |
| `/ax12a/status` | `control_brazo/ArmStatus` | Publica |

**Servicios:**
| Servicio | Tipo | Qué hace |
|----------|------|----------|
| `/ax12a/connect` | Trigger | Abre puerto y hace ping a todos los servos |
| `/ax12a/disconnect` | Trigger | Cierra puerto limpiamente |
| `/ax12a/emergency_stop` | Trigger | Para todos los motores inmediatamente |
| `/ax12a/resume` | Trigger | Reactiva el control después de emergencia |
| `/ax12a/calibrate_start` | Trigger | Entra en modo calibración (bloquea joint_cmd) |
| `/ax12a/calibrate_confirm` | Trigger | Define posición actual como 0° en todos |
| `/ax12a/jog` | ServoCommand | Mueve un servo a ángulo absoluto (funciona en calib) |
| `/ax12a/rescue_pulse` | ServoCommand | Pulso de velocidad abierta para liberar atascados |
| `/ax12a/register_servo` | RegisterServo | Agrega un servo al driver en runtime |

**Estado de implementación:**
| Función | Estado |
|---------|--------|
| Loop de lectura y control de posición | ✅ Completo |
| Publicación `/joint_states` | ✅ Completo |
| Servicios connect/disconnect/estop/resume/calib | ✅ Completo |
| Carga de servos desde YAML | ✅ Completo |
| Publicar `/ax12a/status` | ✅ Completo |
| `_srv_jog` | ✅ Completo |
| `_srv_rescue_pulse` | ✅ Completo |
| `_srv_register_servo` | ✅ Completo |
| Respetar `velocity[]` en `joint_cmd` para vel_pct | ✅ Completo |
| Persistencia de estado en disco (`last_state.yaml`) | 🟢 Opcional |

---

### `ex106_driver_node.py`
**Qué es:** Nodo ROS2. Igual al anterior pero para EX-106+ via RS485.  
**Diferencias con el AX driver:** Puerto `/dev/ttyUSB1`, baudrate 57600, namespace `/ex106/`.

> La lógica interna es idéntica al `ax12a_driver_node`. En el futuro se puede refactorizar en una clase base `DynamixelDriverNode` con herencia.

**Estado de implementación:**
| Función | Estado |
|---------|--------|
| Loop de lectura y control de posición | ✅ Completo |
| Servicios connect/disconnect/estop/resume/calib | ✅ Completo |
| Publicar `/ex106/status` | ✅ Completo |
| `_srv_jog` | ✅ Completo |
| `_srv_rescue_pulse` | ✅ Completo |
| `_srv_register_servo` | ✅ Completo |
| Respetar `velocity[]` en `joint_cmd` | ✅ Completo |
| Persistencia de estado en disco | 🟢 Opcional |

---

### `cinematica_node.py`
**Qué es:** Nodo ROS2. Corre FK continuamente y ofrece IK como servicio.  
**Sin hardware.** Solo recibe números de `/joint_states` y devuelve números.

**Responsabilidades:**
- Mantener el vector `q_actual[5]` fusionando mensajes de AX y EX drivers
- Correr FK en cada actualización y publicar la pose del efector
- Publicar los puntos 3D del brazo para que `simulacion_node` los dibuje
- Responder al servicio `/compute_ik` con los ángulos para una pose objetivo

**Topics:**
| Topic | Tipo | Rol |
|-------|------|-----|
| `/joint_states` | `sensor_msgs/JointState` | Suscribe |
| `/end_effector_pose` | `geometry_msgs/PoseStamped` | Publica |
| `/fk_points` | `sensor_msgs/JointState` | Publica (puntos 3D codificados) |

**Estado de implementación:**
| Función | Estado |
|---------|--------|
| Suscripción y fusión de `/joint_states` | ✅ Completo |
| FK y publicación de `/end_effector_pose` | ✅ Completo |
| Publicación de `/fk_points` | ✅ Completo |
| Servicio `/compute_ik` | ✅ Completo |
| RPY en el quaternion del topic de pose | ✅ Completo |

> **No hay pendientes.** Este archivo está completo y listo.

---

### `simulacion_node.py`
**Qué es:** Nodo ROS2. Visualización 3D del brazo en matplotlib.  
**Sin hardware, sin cálculos.** Solo dibuja lo que recibe.

**Solución al conflicto event loop:**  
ROS spin corre en un thread daemon. Matplotlib (TkAgg) corre en el thread principal,
que es el requerido por Tkinter. `plt.pause(0.12)` cede el control al event loop de Tk
cada 120ms, permitiendo que la ventana responda y que el thread ROS ejecute callbacks.

**Topics:**
| Topic | Tipo | Rol |
|-------|------|-----|
| `/fk_points` | `sensor_msgs/JointState` | Suscribe (posición real) |
| `/sim/fk_points_preview` | `sensor_msgs/JointState` | Suscribe (preview GUI) |

**Parámetros:**
| Parámetro | Default | Qué controla |
|-----------|---------|-------------|
| `reach` | 0.74 | Escala de los ejes de la visualización (metros) |

**Estado de implementación:**
| Función | Estado |
|---------|--------|
| Visualización 3D, frontal y superior | ✅ Completo |
| Overlay de preview en naranja | ✅ Completo |
| Integración event loop (thread separado para ROS) | ✅ Completo |
| `reach` desde parámetros ROS / YAML | ✅ Completo |

> **No hay pendientes.** Este archivo está completo y listo.

---

### `gui_node.py`
**Qué es:** Nodo ROS2 + ventana CustomTkinter. La interfaz de usuario.  
**Sin hardware, sin matemáticas.** Todo lo que necesita lo pide a los otros nodos.

**Responsabilidades:**
- Mostrar ángulos actuales y pose del efector (desde topics)
- Enviar comandos FK (publicar en `/ax12a/joint_cmd` y `/ex106/joint_cmd`)
- Pedir IK al servicio `/compute_ik` y ejecutar el resultado
- Llamar servicios de los drivers para conectar, calibrar, emergencia
- Panel de calibración: mostrar ángulos actuales, botones jog ±, confirmar home
- Panel rescue pulse: seleccionar joint, direccion, disparar pulso

**Diseño thread-safe:**  
ROS gira en un thread daemon. Los callbacks de ROS (IK result, jog result)
guardan datos en variables `_pending_*`. El timer `_loop_ui` (120ms, thread tkinter)
lee esas variables y actualiza la UI.

**Flujo de calibración:**
1. Usuario hace click en "Iniciar calibración" → llama `calibrate_start` en ambos drivers
2. Panel de calibración aparece con ángulos actuales y botones ◄/►
3. Usuario usa botones jog para mover cada joint a la posición física de home
4. Usuario hace click en "CONFIRMAR HOME" → llama `calibrate_confirm` en ambos drivers
5. Panel desaparece, modo normal reanuda

**Topics:**
| Topic | Tipo | Rol |
|-------|------|-----|
| `/joint_states` | `sensor_msgs/JointState` | Suscribe |
| `/end_effector_pose` | `geometry_msgs/PoseStamped` | Suscribe |
| `/ax12a/status` | `control_brazo/ArmStatus` | Suscribe |
| `/ex106/status` | `control_brazo/ArmStatus` | Suscribe |
| `/ax12a/joint_cmd` | `sensor_msgs/JointState` | Publica |
| `/ex106/joint_cmd` | `sensor_msgs/JointState` | Publica |
| `/sim/fk_points_preview` | `sensor_msgs/JointState` | Publica (preview IK) |

**Estado de implementación:**
| Función | Estado |
|---------|--------|
| Mostrar ángulos actuales en tiempo real | ✅ Completo |
| Mostrar X/Y/Z del efector | ✅ Completo |
| Mostrar Roll/Pitch/Yaw del efector | ✅ Completo |
| Control FK (publicar comandos) | ✅ Completo |
| Enviar `vel_pct` al driver vía `velocity[]` | ✅ Completo |
| Control IK async (llamar `/compute_ik`) | ✅ Completo |
| Habilitar/deshabilitar botones según estado real | ✅ Completo |
| Toggle Conectar/Desconectar con estado real | ✅ Completo |
| Panel de calibración (jog + confirmar) | ✅ Completo |
| Panel de pulso de rescate | ✅ Completo |
| Ventana de simulación IK preview | 🟢 Opcional (publica en `/sim/fk_points_preview`, la renderiza `simulacion_node`) |

---

## Resumen de pendientes globales

| Prioridad | Pendiente | Afecta a |
|-----------|-----------|----------|
| ✅ Hecho | Activar servicio `/compute_ik` en `cinematica_node` | — |
| ✅ Hecho | IK async en `gui_node` | — |
| ✅ Hecho | Suscribir `/ax12a/status` y `/ex106/status` en GUI | — |
| ✅ Hecho | Publicar `/ax12a/status` y `/ex106/status` en drivers | — |
| ✅ Hecho | RPY del quaternion en `gui_node` | — |
| ✅ Hecho | Habilitar/deshabilitar botones según estado real | — |
| ✅ Hecho | `CMakeLists.txt` para compilar interfaces custom | — |
| ✅ Hecho | Servicios `jog` y `rescue_pulse` en ambos drivers | — |
| ✅ Hecho | Servicio `register_servo` en ambos drivers | — |
| ✅ Hecho | Panel calibración (jog) en GUI | — |
| ✅ Hecho | Panel rescue pulse en GUI | — |
| ✅ Hecho | Resolver conflicto matplotlib + rclpy en `simulacion_node` | — |
| ✅ Hecho | Escalar `reach` desde YAML en `simulacion_node` | — |
| ✅ Hecho | Enviar `vel_pct` al driver (fix bug: no se enviaba) | — |
| 🟢 Baja | Persistencia de estado en disco (drivers) | Solo necesario si hay reinicios frecuentes |
| 🟢 Baja | Refactorizar AX/EX driver en clase base común | Evitar duplicación de código |
| 🟢 Baja | Ventana preview IK separada en GUI | La GUI ya publica en `/sim/fk_points_preview`; `simulacion_node` la muestra |

---

## Flujo de arranque

```bash
# 1. Compilar el paquete
cd ~/ros2_ws
colcon build --packages-select control_brazo

# 2. Sourcear
source install/setup.bash

# 3a. Con hardware
ros2 launch control_brazo full_system.launch.py

# 3b. Sin hardware (desarrollo)
ros2 launch control_brazo sim_only.launch.py
```

---

## Notas de diseño importantes

### vel_pct en joint_cmd
El campo `velocity[]` del mensaje `sensor_msgs/JointState` se usa para pasar la velocidad
como porcentaje (`vel_pct`) a cada joint. Si el array está vacío, el driver usa el valor
por defecto del encoder (30%). El driver actualiza `enc.vel_pct` antes del próximo ciclo.

### Calibración y modo jog
En modo calibración (`modo_calib=True`):
- `_cb_joint_cmd` ignora todos los mensajes del topic `joint_cmd`
- El servicio `/ax12a/jog` (y `/ex106/jog`) sí funciona — setea `enc.target_deg` directamente
- El loop de control ejecuta `target_deg` independientemente del flag `modo_calib`
- Esto permite mover joints individualmente para ajustar la posición de home

### Rescue pulse
El campo `target_deg` en `ServoCommand.srv` se repurposea en el contexto del rescue pulse:
- `target_deg >= 0` → pulso en dirección positiva
- `target_deg < 0` → pulso en dirección negativa
La velocidad se acota internamente a [5%, 30%] para evitar daños.
