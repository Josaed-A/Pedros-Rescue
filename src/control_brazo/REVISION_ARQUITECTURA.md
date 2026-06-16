# Revision Arquitectonica del Proyecto `control_brazo`

## Objetivo de la revision

Este documento actualiza la revision arquitectonica del proyecto para reflejar el estado real actual del paquete `control_brazo`.

La meta de esta revision es dejar claramente documentado:

- que partes de la arquitectura ya quedaron resueltas
- que problemas reales persisten
- que decisiones de arquitectura siguen pendientes
- que acciones concretas faltan para cerrar reproducibilidad, mantenibilidad y coherencia documental

La evaluacion ya no trata al sistema como una integracion rota o incompleta en lo esencial. El estado actual es mejor descrito como:

- base funcional
- simulacion completa ya integrada
- deuda tecnica y documental todavia abierta

---

## Estado actual resumido

Hoy el proyecto ya implementa una arquitectura dual:

- modo real, con `ax12a_driver_node.py` y `ex106_driver_node.py`
- modo simulado, con `sim_driver_node.py` ejecutado bajo namespaces compatibles

La capa superior del sistema se mantiene estable entre ambos modos:

- `gui_node.py` sigue usando los mismos topics y servicios
- `cinematica_node.py` sigue calculando FK/IK desde `/joint_states`
- `simulacion_node.py` sigue renderizando la geometria a partir de `/fk_points`

El cambio arquitectonico mas importante respecto a la revision anterior es que `sim_only.launch.py` ya no es una simulacion parcial. Ahora levanta un backend simulado compatible con la interfaz publica del sistema real.

Eso significa que el proyecto ya cuenta con:

- una topologia ROS funcional para pruebas end-to-end sin hardware real
- una separacion clara entre backend real y backend simulado
- una GUI que puede operar sobre ambos modos sin cambiar de interfaz publica

---

## Arquitectura real actual

## Modo real

En `full_system.launch.py` se levanta el sistema con hardware real:

- `ax12a_driver`
- `ex106_driver`
- `cinematica`
- `simulacion`
- `gui_control`

En este modo:

- los drivers reales publican `/joint_states`
- la GUI opera via topics y servicios reales
- `cinematica_node.py` calcula FK e IK
- `simulacion_node.py` visualiza el estado del brazo

## Modo simulado

En `sim_only.launch.py` ahora se levanta una topologia paralela:

- `ax12a_sim_driver`
- `ex106_sim_driver`
- `cinematica`
- `simulacion`
- `gui_control`

Los drivers simulados exponen la misma interfaz ROS publica que los drivers reales:

- `/ax12a/joint_cmd`, `/ax12a/status`, `/ax12a/connect`, etc.
- `/ex106/joint_cmd`, `/ex106/status`, `/ex106/connect`, etc.
- `/joint_states` como salida articular agregada

Esto permite que:

- la GUI no tenga que distinguir entre backend real y backend simulado
- `cinematica_node.py` siga desacoplado del origen del estado articular
- el flujo de prueba sin hardware sea comparable al flujo real

## Rol actual de `sim_driver_node.py`

`sim_driver_node.py` es ahora un componente arquitectonico explicito del sistema.

Su responsabilidad es:

- emular drivers AX-12A y EX-106+ sin hardware
- exponer el mismo contrato ROS que los drivers reales
- mover joints de forma gradual hacia targets
- soportar conexion, desconexion, calibracion, emergencia, jog, rescue y registro de servos

No reemplaza la cinematica ni la GUI. Actua como backend operativo alternativo.

---

## Cambios ya resueltos

Esta seccion documenta hallazgos de la revision anterior que ya no deben considerarse problemas abiertos.

## 1. Backend simulado agregado

Ya existe un backend simulado funcional:

- `sim_driver_node.py`
- `ax12a_sim_driver`
- `ex106_sim_driver`

Resultado:

- `sim_only.launch.py` ya permite pruebas end-to-end sin hardware real

## 2. `sim_only.launch.py` ya no es parcial

La revision anterior indicaba que `sim_only` solo levantaba:

- `cinematica`
- `simulacion`
- `gui_control`

Eso ya no aplica.

Ahora `sim_only.launch.py` monta:

- drivers simulados
- cinematica
- simulacion
- GUI

## 3. Imports internos del paquete corregidos

Los imports errados del tipo `control_brazo.control_brazo.*` ya fueron corregidos.

Esto mejora:

- compatibilidad del paquete instalado
- coherencia de entry points
- reproducibilidad de ejecucion

## 4. `simulacion_node.py` ya recibe configuracion desde launch

Los launch files ya pasan configuracion al nodo de simulacion.

Esto cierra el problema previo donde `reach` quedaba solo en defaults locales.

## 5. Preview IK ya operativo

El preview ya no es solo un publisher declarado.

Actualmente:

- la GUI publica `JointState` de preview
- `cinematica_node.py` lo transforma a puntos FK de preview
- `simulacion_node.py` consume `/sim/fk_points_preview`

Resultado:

- el overlay de preview ya existe de punta a punta

## 6. Fix thread-safe del mensaje de rescue

La ruta de rescue ya no actualiza widgets directamente desde callbacks ROS.

Ahora:

- el callback guarda estado pendiente
- `_loop_ui` aplica el cambio desde el thread de Tk

## 7. Dependencias ROS de launch agregadas a `package.xml`

`package.xml` ya incorpora dependencias ROS que faltaban para launch y resolución de recursos:

- `ament_index_python`
- `launch`
- `launch_ros`

Esto corrige una parte importante de la reproducibilidad ROS del paquete.

---

## Problemas persistentes

Solo se listan aqui problemas reales que siguen abiertos en el estado actual del repo.

## Problema 1. Configuracion critica todavia hardcodeada en la GUI

### Descripcion

La GUI sigue manteniendo configuracion operativa en codigo:

- `JOINT_ORDER`
- `JOINT_SERVO`

Eso fija en el codigo:

- nombres de joints
- orden de joints
- IDs de servo
- asignacion AX o EX por joint

### Impacto

Si cambia la configuracion de `servos.yaml`:

- la GUI puede quedar desalineada
- el comportamiento deja de depender de una sola fuente de verdad
- se rompe la promesa de configuracion centralizada

### Gravedad

Alta.

No bloquea funcionamiento actual, pero si mantiene fragilidad estructural.

### Solucion recomendada

- mover orden, nombres, IDs y tipo de driver a configuracion compartida
- hacer que la GUI derive sus controles, mapping y logica desde esa fuente

---

## Problema 2. Reproducibilidad Python todavia incompleta

### Descripcion

Aunque `package.xml` mejoro, `setup.py` sigue declarando:

- `install_requires=['setuptools']`

Las dependencias Python operativas siguen fuera del contrato ejecutable formal del proyecto.

Entre ellas:

- `numpy`
- `matplotlib`
- `customtkinter`
- `dynamixel_sdk`

### Impacto

El entorno sigue dependiendo de instalacion manual documentada y no de una entrada unica y reproducible.

### Gravedad

Media-alta.

No impide el uso local si se sigue la documentacion, pero si deja debil la reproducibilidad completa.

### Solucion recomendada

- definir una estrategia explicita de dependencias Python
- crear un `requirements.txt` global en la raiz del repo
- usar ese archivo como punto de entrada para preparar entorno de desarrollo y simulacion

---

## Problema 3. `ARQUITECTURA.md` esta desactualizado

### Descripcion

La documentacion arquitectonica principal todavia describe un sistema anterior.

Problemas concretos:

- sigue hablando de un sistema de 5 nodos
- no incorpora `sim_driver_node.py`
- `sim_only` todavia aparece como "solo cinematica + visualizacion + GUI"
- varias descripciones quedaron atrasadas respecto a la implementacion actual

### Impacto

La documentacion ya no refleja la arquitectura real que el repo implementa hoy.

### Gravedad

Alta.

Es una fuente de confusion para mantenimiento, onboarding y validacion de alcance.

### Solucion recomendada

- actualizar `ARQUITECTURA.md` para reflejar modo real y modo simulado
- actualizar el diagrama topologico
- documentar explicitamente el rol del driver simulado
- revisar la seccion de pendientes para que coincida con el estado actual

---

## Problema 4. `SETUP_SIMULACION.md` quedo atrasado respecto al sistema actual

### Descripcion

La guia de simulacion todavia describe un flujo anterior.

Problemas concretos:

- sigue diciendo que `sim_only` levanta solo 3 nodos
- no menciona drivers simulados
- sigue proponiendo publicar `/joint_states` a mano como flujo principal
- no describe el flujo actual completo con GUI y backend simulado

### Impacto

La guia no acompaña la capacidad real que hoy ya tiene el sistema.

### Gravedad

Alta.

No es un bug del software, pero si un bug operativo de adopcion y uso correcto.

### Solucion recomendada

- actualizar instrucciones de `sim_only`
- listar todos los nodos reales que ahora se levantan
- explicar el flujo esperado con GUI, drivers simulados y cinematica
- dejar la publicacion manual por topic solo como prueba opcional, no como flujo central

---

## Problema 5. Duplicacion estructural entre drivers reales AX y EX

### Descripcion

`ax12a_driver_node.py` y `ex106_driver_node.py` siguen compartiendo gran parte de su logica.

### Impacto

Esto mantiene:

- deuda tecnica
- riesgo de divergencia
- costo doble al corregir bugs o extender funcionalidades

### Gravedad

Media.

No es bloqueo funcional actual, pero si una deuda tecnica importante.

### Solucion recomendada

- extraer una clase base o capa comun
- mover logica compartida fuera de cada nodo concreto
- hacerlo despues de estabilizar configuracion y documentacion

---

## Problema 6. La capa simulada ya es funcional, pero la estrategia de configuracion no esta unificada

### Descripcion

El backend simulado ya cumple su interfaz publica y ya aporta valor real.

Sin embargo, la arquitectura de configuracion aun no esta unificada de extremo a extremo entre:

- GUI
- drivers reales
- drivers simulados
- cinematica

### Impacto

La arquitectura general mejoro mucho, pero todavia no hay una fuente de verdad completamente compartida para toda la capa superior y ambos backends.

### Gravedad

Media.

Es una observacion arquitectonica mas que un bug de funcionamiento inmediato.

### Solucion recomendada

- usar una misma fuente de verdad de configuracion para real y simulado
- alinear esa fuente con la GUI y con la construccion de controles

---

## Soluciones recomendadas por prioridad

## Prioridad 1. Alinear documentacion con la implementacion real

Acciones:

- actualizar `ARQUITECTURA.md`
- actualizar `SETUP_SIMULACION.md`
- reflejar la existencia de la arquitectura dual real/simulada
- actualizar flujos de arranque y descripciones de nodos

Motivo:

- hoy la principal fuente de error humano ya no es una falla de codigo, sino documentacion atrasada

## Prioridad 2. Sacar configuracion critica de la GUI

Acciones:

- eliminar hardcode de `JOINT_ORDER`
- eliminar hardcode de `JOINT_SERVO`
- leer orden, IDs, nombres y tipo de driver desde una configuracion compartida

Motivo:

- esto ataca la fragilidad estructural mas importante que sigue abierta

## Prioridad 3. Cerrar reproducibilidad Python

Acciones:

- crear `requirements.txt` global en la raiz
- mover la documentacion a una estrategia unica de instalacion Python
- decidir si `setup.py` seguira minimo y el contrato de entorno vivira en `requirements.txt`, o si ambos se alinearan despues

Motivo:

- la reproducibilidad todavia depende demasiado de pasos manuales dispersos

## Prioridad 4. Unificar estrategia de configuracion entre real y simulado

Acciones:

- definir una fuente de verdad compartida
- hacer que GUI, drivers reales, drivers simulados y cinematica lean del mismo modelo logico

Motivo:

- el backend simulado ya existe; ahora falta cerrar coherencia total

## Prioridad 5. Reducir duplicacion AX/EX

Acciones:

- extraer base comun o capa compartida
- dejar solo diferencias reales en nodos concretos

Motivo:

- mejora de mantenimiento y evolucion futura

---

## Decisiones de arquitectura pendientes

Estas decisiones siguen abiertas a nivel de diseño, aunque el sistema ya funcione mejor.

## 1. Fuente de verdad de configuracion

Queda pendiente decidir formalmente si toda la configuracion compartida se centraliza en:

- `servos.yaml` actual
- o una capa/configuracion adicional derivada

La recomendacion actual es mantener una unica fuente de verdad legible por:

- GUI
- drivers reales
- drivers simulados
- cinematica

## 2. Politica de dependencias Python

Debe quedar definido si la estrategia oficial sera:

- `requirements.txt` global como contrato del entorno
- `setup.py` solo para empaquetado ROS/Python del paquete

La recomendacion actual es:

- `package.xml` para dependencias ROS
- `requirements.txt` global para dependencias Python del entorno

## 3. Estrategia de refactor de drivers

Queda pendiente definir si la unificacion de AX/EX se hara mediante:

- clase base
- mixin comun
- modulo compartido de logica de control

No bloquea el estado actual, pero conviene dejarlo explicitado cuando se ataque deuda tecnica.

---

## Reproducibilidad del entorno

## Estado actual

La reproducibilidad del entorno mejoro, pero aun esta repartida entre:

- `package.xml`
- `setup.py`
- instalacion manual via `pip`
- pasos documentales en `SETUP_SIMULACION.md`

## Recomendacion arquitectonica actual

Separar con claridad:

- dependencias ROS del paquete
- dependencias Python del entorno

Quedaria asi:

- `package.xml` mantiene dependencias ROS
- `requirements.txt` global define dependencias Python necesarias para correr el repo

Esto simplifica:

- onboarding
- entornos nuevos
- simulacion
- automatizacion futura

---

## Accion adicional: `requirements.txt` global en la raiz

## Requerimiento nuevo

Se recomienda crear un archivo de dependencias Python global en la raiz del repo:

- [requirements.txt](/c:/Users/juanu/Documents/Rescue/requirements.txt)

## Motivo

El repo tiene dependencias Python que no pertenecen solo al empaquetado interno ROS del paquete.

Hoy la preparacion del entorno depende de:

- texto suelto en documentacion
- instalacion manual
- conocimiento implicito de que paquetes Python hacen falta

Un archivo global corrige eso.

## Contenido esperado minimo

El archivo debe listar, como minimo:

- `numpy`
- `matplotlib`
- `customtkinter`
- `dynamixel_sdk`

## Politica recomendada

- `package.xml` sigue declarando dependencias ROS
- `requirements.txt` pasa a ser la fuente de verdad para dependencias Python del entorno
- `SETUP_SIMULACION.md` debe instalar desde ese archivo global en vez de enumerar paquetes Python sueltos

---

## Conclusiones actualizadas

La revision anterior ya no describe bien el estado del proyecto.

Hoy `control_brazo` ya no esta en una situacion de integracion critica abierta. El cambio mas importante es que la simulacion funcional completa ya existe y ya forma parte real de la arquitectura.

El estado actual del proyecto puede resumirse asi:

- la base funcional esta bien encaminada
- el modo real y el modo simulado ya conviven bajo una interfaz ROS consistente
- los problemas mas fuertes pasaron de ser faltantes estructurales a deuda tecnica y documentacion desalineada

Las prioridades reales ahora son:

1. actualizar documentacion principal
2. eliminar configuracion hardcodeada de la GUI
3. cerrar reproducibilidad Python con `requirements.txt` global
4. unificar la estrategia de configuracion entre real y simulado
5. reducir duplicacion de drivers reales

La arquitectura ya dio el salto importante: ahora falta consolidarla.
