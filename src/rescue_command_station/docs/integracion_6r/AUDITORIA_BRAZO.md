# Auditoría del control del brazo — 2026-09-10

## Resultado y alcance

La integración anterior sustituyó el solver, pero **no había eliminado todos
los componentes heredados**. Esta revisión retira los restos activos y corrige
los problemas de integración encontrados. No se puede afirmar que el sistema
esté validado en ROS o en hardware: ambos quedan pendientes en este entorno.

Revisados: módulos `arm`, consumidores del dashboard, lanzadores PC/Pi, interfaces,
YAML, empaquetado, script de arranque del contenedor, simulador de drivers y
fronteras con drivers reales/encoder. Se conservaron los cambios locales y las
cinco medidas geométricas originales. Sin commit, push, ejecución ROS ni motores.

El núcleo `kinematics.py`, `motion.py`, `cartesian_controller.py`, el laboratorio
Python entregado y los 12 archivos del web original se conservan byte a byte
respecto al ZIP. No se reimplementó FK, IK, Jacobiano, SLERP ni el planificador.
El dibujo integrado sigue usando el Canvas extraído de la entrega.

## Hallazgos corregidos

| Hallazgo | Corrección y comprobación |
| --- | --- |
| Visor Matplotlib anterior y transporte de puntos aún presentes | Retirados `simulacion_node.py`, entrada `arm_sim_node`, `/fk_points`, `/sim/fk_points_preview`, sus suscriptores y el preview ROS heredado. El Canvas usa snapshots del modelo compartido. |
| Orientación reducida cruzada con el nuevo modelo | Retirados `ComputeIK.srv` y `/compute_ik`. GUI usa XYZ/RPY y matriz completa. Reproducción: q modelo = [20,50,-40,-10,25,15] grados cambiaba casi 180° al reconstruir desde beta/q5/q6; la regresión RPY conserva R. |
| Botones que resolvían solo el destino y podían emitir órdenes diferidas | Los incrementos y movimientos cartesianos validan todo el recorrido. IK aislada dibuja preview y nunca guarda una orden para ejecutar luego. |
| Estados independientes de “IK ocupado” y mensajes usados como lógica | Una reserva de petición en GUINode y el estado del controlador gobiernan disponibilidad; se retiraron flags de joystick/órdenes pendientes redundantes. El controlador conserva la admisión central. |
| Configuración duplicada y precedencia incorrecta entre fuente/instalación | Cargador único por nodo, snapshot inmutable, parámetros de solo lectura y rutas explícitas compartidas en launch. En ROS se consulta el overlay instalado. |
| Límites invertidos aceptados por ordenación silenciosa | Se valida min < max antes de transformar signos. Sliders respetan el YAML y la copia de ángulos no aplica vueltas silenciosas. |
| Mapa de servos incluía entradas deshabilitadas y ocultaba IDs ausentes | Se usan los `servo_names` activos como los drivers; se comprueban seis nombres, cobertura exacta, duplicados, IDs y colisiones brazo/patas. Se retiran fallbacks de ID 0. |
| Simulación publicaba 0..360° y hardware reporta ángulos con signo | El simulador convierte únicamente el feedback al intervalo [-180,180); se conserva su cálculo circular interno. Regresión: 350° se publica como -10°. |
| Mantenimiento podía competir con movimientos de estación | Servicio `/cartesian/maintenance` bloquea admisión y publicación bajo el mismo cerrojo. GUI adquiere antes de calibrar y libera después de confirmar ambos buses. Fallo parcial deja el bloqueo. Jog/rescate requieren ambos buses en calibración, recientes y sin emergencia. |
| Estado antiguo presentado como actual | Pose, seis articulaciones y buses caducan; el controlador publica `feedback_valid`, `maintenance` y XYZ NaN sin feedback completo. GUI distingue falta de estado y bloqueo. |
| Mando desconectado podía conservar ejes activos | No se generan nuevos segmentos si el último Joy tiene más de 0.7 s. Un segmento ya aceptado puede terminar; no se presenta como parada de emergencia. |
| Estabilización cambiaba orientación incluso si se rechazaba el plan | El objetivo fijo se confirma solo tras aceptación. Los ejes relativos usan la orientación medida del TCP activo. |
| Diccionario de telemetría compartido entre ROS y Tk | Intercambio de filas protegido por cerrojo; Tk procesa un snapshot. |
| GUI independiente arrancaba oculta esperando dashboard | Visible por defecto; dashboard pasa `managed_by_dashboard:=true`. Cierre independiente termina su launch mediante evento, sin matar el grupo de la terminal. |
| Arranque independiente debía conservar la exclusión con teleop de base | Publica `/arm_active` mientras está abierta y lo libera al cerrar normalmente. Se revisó `ps4_teleop_node`: inhibe la base y no publica comandos de brazo. |
| Dos definiciones de lanzamiento de drivers Pi | `pi_sensors` incluye el launch canónico `rescue_robot_core/servos.launch.py`. Las configuraciones personalizadas se pasan con `servos_config`. |
| Contenedor instalaba dependencias viejas y ocultaba errores de build con `tail` | Usa `requirements_pc.txt`, instala Chromium y construye `--packages-up-to rescue_command_station`. Eliminada la limpieza destructiva de `mpl_toolkits`. Script fijado a LF. |
| Simulador sin servicios esperados por la GUI y sin cierre de hilo | Añadidos `servo_status`/`reset_alerts`, identificados como simulación; cierre mediante evento y join. El rescate simulado también rechaza emergencia. |
| Comentarios e interfaces describían el comportamiento antiguo | Documentados preflight sin respuesta inmediata, duración total 0.2..120 s y convención driver de la respuesta IK. |

## Fuente de cada dato

| Dato | Fuente autoritativa y adaptación |
| --- | --- |
| Geometría/TCP/límites/signos | Un `arm.yaml` compartido por launch; `configuration.py` carga y valida. No se leen YAML al importar la GUI. |
| Orden de J1..J6 | `joint_order` de ese perfil. |
| Bus e ID de cada joint | `servo_names` y `servos` del `servos.yaml` activo. El PC necesita el mismo contenido que la Pi. |
| Posición medida | `/arm/joint_states` aportado por los dos buses. Conversión única `q_model = q_driver * model_sign`. |
| Pose TCP/herramienta/cámara | FK entregada sobre esas articulaciones. Pose completa para IK. |
| Planificación y tolerancias | `motion.py` y `cartesian_controller.py` del ZIP, usados por el adaptador ROS. |
| Ocupación y mantenimiento | Controlador cartesiano; GUI conserva además la reserva de solicitudes aún sin respuesta. |
| Preview | Datos locales de dibujo; no un topic de comando ni una pose medida. |
| Dibujo | Canvas original; sin un segundo solver JavaScript en la GUI ROS. |

El web original independiente sí contiene su implementación JavaScript original.
Se conserva por petición expresa de usar la entrega sin rediseñarla; no se carga
en el control ROS. `component_target` sigue dentro del núcleo intacto del ZIP,
pero no tiene consumidores activos. La rama histórica `test/cinematica` y los
documentos de procedencia conservan el pasado; no son rutas ejecutables activas.

## Pendientes abiertos y límites de la auditoría

1. **Alta prioridad — rescate directo en drivers reales.** En
   `dynamixel_bus_node.py::_srv_rescue_pulse` y
   `ex106_driver_node.py::_srv_rescue_pulse`, la entrada comprueba conexión/servo,
   pero no `_emergencia`, y escribe velocidad durante el pulso de 0.25 s.
   La GUI lo impide según estado reciente; un cliente externo o una carrera con
   emergencia todavía puede llegar a esos servicios. Resolver y probar esto exige
   una intervención separada en el control de bajo nivel, excluido por el alcance
   original. No se considera corregido por añadir una guarda en la GUI.
2. **Alta prioridad — validación ROS pendiente.** Regenerar interfaces en un
   overlay limpio, probar DDS/QoS y comprobar un único proveedor por servicio.
   La retirada de `ComputeIK` no borra artefactos generados de instalaciones
   anteriores. No lanzar simultáneamente otro `arm_station` y el precargado por
   dashboard, ni sim y drivers reales en el mismo dominio.
3. **Frontera de arbitraje.** Publicadores y servicios externos de drivers evitan
   el controlador. El bloqueo de mantenimiento no es persistente entre reinicios
   ni autoriza reanudar automáticamente. Una respuesta incierta mantiene la GUI
   bloqueada; revisar los nodos antes de reiniciar. Confirmar home cambia ceros,
   no es una liberación inocua de bloqueo.
4. **Trayectorias físicas y vueltas.** Los drivers reales usan error circular y
   objetivos articulares; no son controladores temporizados de trayectoria.
   Verificar pasos por ±180°, signos, límites físicos y seguimiento. Los comandos
   manuales/home siguen siendo objetivos articulares directos admitidos por el
   controlador; no implican un recorrido cartesiano ni una confirmación de llegada.
   Cancelar/pausar el envío no cancela el objetivo ya almacenado en el driver.
5. **Medidas y prestaciones pendientes.** Se conservan L3=0 y tool_length=0
   locales; offsets/cámara provisionales. Faltan mediciones, calibración, prueba
   física y comprobación de latencia/CPU de Canvas→PNG→Tk en el PC de destino.
   No hay validación de colisiones, dinámica, carga o control de fuerza.
6. **Simulación limitada.** No modela alarmas físicas, voltaje, temperatura,
   dinámica ni patas del bus AX. Los ceros de telemetría llevan advertencia SIM;
   no son mediciones. No sirve para certificar emergencia, torque o tiempos reales.

## Evidencia reproducible

- 35 pruebas unitarias/regresiones offline: núcleo, ejecución con transporte
  falso, configuración, orientación, preview, mantenimiento, joystick y simulación.
- `tools/verify_arm_gui.py`: construye la GUI real y recorre todas sus pestañas
  con transporte falso y ventana oculta; comprueba copia de pose y cierre.
- `tools/verify_arm_canvas.py --tk`: Chromium, gestos, snapshot y widget Tk.
- Comprobación estática de definiciones/referencias activas, interfaces y entry points;
  análisis `pyflakes` de adaptadores; compilación sintáctica y `git diff --check`.
- Comparación con ZIP de los motores y del web; comparación de drivers reales
  contra `test/cinematica`: únicamente el heartbeat añadido en la integración.

Estas comprobaciones no ejecutan servicios ROS ni hardware. Véase
[VALIDACION.md](VALIDACION.md) para el procedimiento ROS pendiente y
[ARM_6R.md](../../ARM_6R.md) para uso y configuración.
