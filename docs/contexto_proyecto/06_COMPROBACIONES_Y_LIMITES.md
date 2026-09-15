# Comprobaciones y límites de la auditoría

## Método

Auditoría estática general de organización, paquetes, entry points, launch, interfaces, configuración, drivers, cámaras/percepción, UI, misión, exportación, entrenamiento y despliegue. Inventario estructural de los 167 archivos versionados; lectura dirigida de las implementaciones y cadenas relevantes. No equivale a demostrar cada rama de cada archivo ni a aprobar el robot para operar.

Documentación previa se trató como hipótesis. Las conclusiones están respaldadas por código/rutas; en el informe se separan fallos reproducidos, contradicciones estáticas y riesgos que exigen ejecución. No se descargaron reglas de competición ni se verificaron afirmaciones externas de rendimiento o compatibilidad de SDK.

## Resultados obtenidos

| Comprobación | Resultado | Evidencia |
|---|---|---|
| Estado inicial | Commit 3b755cec99230810fd10f5776eb4522d82439095, dos archivos previamente modificados y docs previas untracked | 00_LEER_PRIMERO.md y estado_inicial_sha256.json |
| Archivos versionados | 167; 87 Python con AST válido; siete package.xml | 04_INVENTARIO_ARCHIVOS.md, sintaxis.txt |
| Contratos ROS | Índice de llamadas publisher/subscription/service/client y expresiones de nombres | 05_CONTRATOS_ROS.md; no incluye como llamadas create_subscription las suscripciones creadas con message_filters |
| Launch | 15 archivos: 11 bringup, 2 core, 2 estación | 07_LAUNCH.md |
| Configuración | Cuatro YAML, RViz y XML Xacro parsean; no se expandió Xacro ni se cargaron nodos | config_sintaxis.txt |
| Shell | bash -n pasa en los seis scripts | shell_sintaxis.txt; no se ejecutan sus acciones |
| Canvas generado | Coincide exactamente con salida del generador actual | canvas_generado.txt; mkdir/write_text interceptados en memoria |
| Pruebas existentes, suite completa | Collection bloqueada por tkinter en test_station_integration | pruebas_existentes.txt |
| Pruebas existentes, otros tres módulos | 15 passed, 10 errors por falta de tkinter en setUpClass | pruebas_subconjunto.txt; 15 casos incluyen cinemática y adaptador cartesiano falso |
| Fallos aislados | Cinco reproducciones exitosas: joystick tras desconexión, falta 32FC1, padding imagen, padding nube, stamp cero de scan | reproducir_hallazgos.py y reproducciones.txt |
| Sector LiDAR | Comprobación geométrica de yaw pi y rayo cero → -X de base | reproducciones.txt; no es medición del montaje real |
| Preservación | Los 167 SHA256 versionados coinciden con el inicio de esta tarea | integridad_final.txt |

## Entorno de pruebas y reproducción

El Python activo es 3.13. Existe /opt/ros/jazzy, pero ros2/colcon no están en PATH de la sesión; rclpy y las dependencias del proyecto no son importables desde el Python activo. build/install fueron generados para Python 3.12 según CMakeCache y rutas de extensiones. No se probó que cargar setup.bash resuelva esa diferencia; tampoco se modificó el entorno global.

Se creó únicamente un venv temporal en `/tmp/pedros-audit-20260912` para instalar numpy, PyYAML y pytest. Versiones exactas en entorno_pruebas.txt y registro de instalación en dependencias_prueba.txt. Este venv puede desaparecer y no es una dependencia nueva del proyecto. No se instaló ROS, Tkinter ni Chromium. Las pruebas deshabilitaron bytecode y cache pytest.

Desde la raíz del proyecto, con ese venv disponible:

```bash
PYTHONDONTWRITEBYTECODE=1 /tmp/pedros-audit-20260912/bin/python docs/contexto_proyecto/reproducir_hallazgos.py

PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/rescue_command_station:src/rescue_robot_core /tmp/pedros-audit-20260912/bin/python -m pytest -q -p no:cacheprovider src/rescue_command_station/test
```

Para reproducir el subconjunto ejecutado, sustituir el último directorio por los tres archivos test_arm_6r.py, test_arm_audit.py y test_executor_logic.py. En este entorno se esperan los diez errores de preparación por Tkinter; no deben contarse como fallos de la lógica de esos tests.

Las reproducciones anexas extraen métodos mediante AST y usan datos/transporte falsos. Demuestran el comportamiento concreto indicado; no arrancan nodos, no conectan GPIO/USB ni simulan íntegramente ROS. El primer intento del arnés de joystick necesitó completar el atributo dev del objeto falso; el resultado conservado corresponde al arnés corregido, sin modificar la implementación auditada.

## Pendiente de validación en el entorno apropiado

- Compilación limpia colcon en Python/ROS compatibles y prueba de instalación sin symlinks al checkout anterior.
- Grafo real de todos los perfiles launch, herencia de argumentos, nombres duplicados y QoS; sin conectar actuadores inicialmente.
- Suite completa con Tkinter, verificación visual de GUIs/Chromium y verificadores JavaScript con Node.
- Simulación de pérdidas de mando, paquete DDS, lectura de servo y shutdown; después banco físico controlado para verificar que la parada llega al hardware.
- Inventario real de Pi, commit desplegado, dispositivos, reglas udev y contenedores/autostart sin terminal.
- Calibración de cámara/profundidad/TF, orientación del LiDAR, geometría y límites mecánicos.
- Misión completa con arranque/parada/reinicio, manuales y automáticos, exportación y relectura consistente de CSV/PLY/TIFF.
- Calidad y procedencia de pesos/dataset, paridad PT/ONNX, memoria/CPU/latencia en PC y Pi.
- Revisión de requisitos externos de competición si se solicita validar cumplimiento; esta auditoría no lo certifica.

No se ejecutaron contenedores, scripts de instalación/autostart, SSH/scp, nodos ROS, entrenamientos, cargas de pesos ni operaciones físicas. No se aplicaron recomendaciones de limpieza o arquitectura.
