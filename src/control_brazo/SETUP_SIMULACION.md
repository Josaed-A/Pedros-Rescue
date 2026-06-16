# Setup — Simulación sin hardware (Linux + ROS2)

## Requisitos previos

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.10+

---

## 1. Instalar ROS2 Jazzy

Si no está instalado:

```bash
# Configurar locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Agregar repositorio ROS2
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions
```

---

## 2. Dependencias del sistema (apt)

```bash
sudo apt install -y \
    python3-pip \
    python3-tk \
    python3-numpy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-launch-ros \
    ros-jazzy-ament-cmake-python \
    ros-jazzy-ament-index-python
```

---

## 3. Dependencias Python (pip)

```bash
pip3 install --user \
    matplotlib \
    customtkinter \
    numpy
```

> **Nota:** `customtkinter` es para la GUI. Si solo querés probar la simulación
> visual sin GUI, no es estrictamente necesario en este paso.

---

## 4. Crear workspace y copiar el paquete

```bash
mkdir -p ~/ros2_ws/src
# Copiar la carpeta control_brazo (la que tiene package.xml) dentro de src/
cp -r /ruta/a/control_brazo ~/ros2_ws/src/
```

La estructura debe quedar así:

```
~/ros2_ws/
└── src/
    └── control_brazo/          ← esta carpeta tiene package.xml
        ├── package.xml
        ├── CMakeLists.txt
        ├── setup.py
        ├── setup.cfg
        ├── config/
        │   └── servos.yaml
        ├── launch/
        │   ├── sim_only.launch.py
        │   └── full_system.launch.py
        ├── msg/
        │   └── ArmStatus.msg
        ├── srv/
        │   ├── ComputeIK.srv
        │   ├── RegisterServo.srv
        │   └── ServoCommand.srv
        ├── resource/
        │   └── control_brazo
        └── control_brazo/      ← código Python
            ├── __init__.py
            ├── kinematics.py
            ├── wheel_encoder.py
            ├── cinematica_node.py
            ├── simulacion_node.py
            ├── gui_node.py
            ├── ax12a_driver_node.py
            ├── ex106_driver_node.py
            └── sim_driver_node.py
```

---

## 5. Compilar el paquete

```bash
cd ~/ros2_ws

# Sourcear ROS2 base (hacer esto en cada terminal nueva)
source /opt/ros/jazzy/setup.bash

# Compilar
colcon build --packages-select control_brazo

# Sourcear el workspace compilado
source install/setup.bash
```

Si hay errores de compilación de interfaces custom (msg/srv), verificar que
`CMakeLists.txt` incluya `rosidl_generate_interfaces`.

---

## 6. Lanzar la simulación (sin hardware)

```bash
# Abrir nueva terminal y sourcear
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Lanzar: drivers simulados + cinematica + simulacion + GUI
ros2 launch control_brazo sim_only.launch.py
```

Esto levanta **5 nodos**:
- `ax12a_driver` — driver simulado AX-12A (mismos topics/servicios que el real)
- `ex106_driver` — driver simulado EX-106+ (mismos topics/servicios que el real)
- `cinematica` — calcula FK/IK con parámetros de `servos.yaml`
- `simulacion` — ventana matplotlib con 3 vistas del brazo
- `gui_control` — ventana CustomTkinter de control

La GUI funciona exactamente igual que con hardware real: el botón **Conectar**
activa los drivers simulados, y todos los flujos (FK, IK, calibración, jog,
rescue, E-STOP) están operativos.

---

## 7. Verificar que los nodos están corriendo

En una terminal separada:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Ver nodos activos
ros2 node list

# Debe mostrar:
# /ax12a_driver
# /ex106_driver
# /cinematica
# /simulacion
# /gui_control
```

```bash
# Ver topics activos
ros2 topic list

# Debe mostrar entre otros:
# /joint_states
# /fk_points
# /end_effector_pose
# /ax12a/status
# /ex106/status
# /ax12a/joint_cmd
# /ex106/joint_cmd
# /joint_states_preview
# /sim/fk_points_preview
```

---

## 8. Prueba manual: publicar joint_states a mano

Si querés mover el brazo en la simulación sin usar la GUI:

```bash
# Publicar una posición de prueba (todos en 0 rad)
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
  '{name: ["Base","Hombro","Codo","Munieca_P","Munieca_Y"], \
    position: [0.0, 0.3, -0.5, 0.2, 0.0]}'
```

Deberías ver el brazo moverse en la ventana de simulación.

---

## 9. Configuración centralizada

Toda la configuración del sistema vive en `config/servos.yaml`:

- **`ax12a_driver`** — servos AX-12A: IDs, reducción, deadband, puerto
- **`ex106_driver`** — servos EX-106+: mismos campos
- **`cinematica`** — parámetros geométricos del brazo (L1, L2, L3, etc.)
- **`simulacion`** — alcance visual (`reach`)
- **`gui_control`** — orden de joints y mapeo joint → driver/ID

Si cambiás un ID de servo, un nombre de joint o la distribución AX/EX,
solo hay que editar el YAML. No hay que tocar el código.

---

## 10. Problemas comunes

### "No module named 'customtkinter'"
```bash
pip3 install customtkinter
```

### "No module named 'control_brazo'"
Olvidaste sourcear el workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

### La ventana de matplotlib no aparece / crash con Tk
Instalar tkinter para Python:
```bash
sudo apt install python3-tk
```
Si estás en sesión SSH sin display, necesitás X11 forwarding o un display virtual:
```bash
sudo apt install xvfb
export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 &
```

### Error al compilar interfaces (msg/srv)
```bash
sudo apt install -y ros-jazzy-rosidl-default-generators
colcon build --packages-select control_brazo --cmake-clean-cache
```

### "package 'control_brazo' not found" en ros2 launch
```bash
# Verificar que está instalado
ros2 pkg list | grep control_brazo
# Si no aparece, recompilar y re-sourcear
colcon build --packages-select control_brazo
source install/setup.bash
```

### Los nodos se cierran solos al cerrar la ventana matplotlib
Normal — cuando cerrás la ventana de simulación, el nodo `simulacion` termina.
El launch file cierra todos los nodos asociados.

### La GUI dice "Desconectado" pero el launch ya levantó todos los nodos
Presioná el botón **Conectar**. Los drivers simulados responden inmediatamente
y la GUI pasa a estado operativo. Esto es igual al flujo real con hardware.

---

## Resumen rápido (si ROS2 ya está instalado)

```bash
pip3 install matplotlib customtkinter numpy
sudo apt install python3-tk

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select control_brazo
source install/setup.bash

ros2 launch control_brazo sim_only.launch.py
```
