# Como ejecutar Pedro's Rescue

Esta guia asume que el repo ya esta en ambas maquinas y que ROS 2 esta instalado.

Reemplaza `<distro>` por tu version de ROS 2, por ejemplo `humble`, `jazzy` o la que esten usando.

## 1. Configurar red ROS 2

En la PC y en la Raspberry usa el mismo `ROS_DOMAIN_ID`:

```bash
export ROS_DOMAIN_ID=10
```

Si lo quieres dejar permanente, agregalo al `~/.bashrc` en ambas maquinas.

## 2. Compilar el workspace

Ejecuta esto en la PC y en la Raspberry:

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Ejecutar en la Raspberry

La Raspberry controla los motores y escucha `/cmd_vel`.

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_robot_core motor_driver_node
```

## 4. Ejecutar en la PC

### Terminal 1: control PS4

```bash
source /opt/ros/<distro>/setup.bash
ros2 run joy joy_node
```

### Terminal 2: teleoperacion

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_command_station ps4_teleop_node
```

### Terminal 3: dashboard opcional

```bash
cd ~/Pedros-Rescue
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 run rescue_command_station dashboard_node
```

## 5. Controles

- Joystick izquierdo hacia adelante: ambas orugas avanzan.
- Joystick izquierdo hacia atras: ambas orugas retroceden.
- Joystick izquierdo hacia un lado: giro sobre el eje.
- Joystick izquierdo en diagonal: una oruga va mas rapido que la otra.
- `R1`: subir caja.
- `L1`: bajar caja.

## 6. Verificaciones utiles

Ver si el control publica:

```bash
ros2 topic echo /joy
```

Ver comandos de velocidad:

```bash
ros2 topic echo /cmd_vel
```

Ver velocidad real reportada por la Raspberry:

```bash
ros2 topic echo /real_speed_abs
```

Ver nodos activos:

```bash
ros2 node list
```

Ver topicos activos:

```bash
ros2 topic list
```

## 7. Orden recomendado de arranque

1. Raspberry: `motor_driver_node`.
2. PC: `joy_node`.
3. PC: `ps4_teleop_node`.
4. PC: `dashboard_node`, si se quiere monitorear.

Si `/cmd_vel` deja de llegar a la Raspberry por mas de 1 segundo, el nodo de motores detiene las salidas por seguridad.
