#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# run_slam_container.sh
# Lanza el stack SLAM de Pedro's Rescue dentro de un contenedor
# ROS 2 Jazzy con acceso al lidar LD19 y la cámara Orbbec.
#
# Uso:
#   ./scripts/run_slam_container.sh           → shell interactivo
#   ./scripts/run_slam_container.sh build     → compilar workspace
#   ./scripts/run_slam_container.sh slam      → lanzar SLAM
#   ./scripts/run_slam_container.sh lidar     → solo lidar + rviz2
# ─────────────────────────────────────────────────────────────────

set -e

WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="pedros-rescue-ros2:jazzy"
CONTAINER="pedros_slam"

# ── Modo exec: conectarse al contenedor ya corriendo ─────────────
# Uso: ./scripts/run_slam_container.sh exec "ros2 topic list"
if [ "${1:-}" = "exec" ]; then
    shift
    EXEC_CMD="${*:-bash}"
    podman exec -it "$CONTAINER" bash -c \
        "source /opt/ros/jazzy/setup.bash && \
         source /workspace/install/setup.bash 2>/dev/null || true && \
         $EXEC_CMD"
    exit $?
fi

# ── Modo diag: diagnóstico completo del sistema ROS ──────────────
if [ "${1:-}" = "diag" ]; then
    podman exec -it "$CONTAINER" bash -c \
        "source /opt/ros/jazzy/setup.bash && \
         source /workspace/install/setup.bash 2>/dev/null || true && \
         echo '=== TOPICS ===' && ros2 topic list && \
         echo '' && echo '=== /ldlidar_node/scan (1 msg) ===' && \
         timeout 5 ros2 topic echo /ldlidar_node/scan --once 2>&1 | head -10 || true && \
         echo '' && echo '=== NODOS ===' && ros2 node list && \
         echo '' && echo '=== INFO SLAM_TOOLBOX ===' && \
         ros2 node info /slam_toolbox 2>&1 || echo 'slam_toolbox no encontrado' && \
         echo '' && echo '=== TF FRAMES ===' && \
         timeout 3 ros2 run tf2_tools view_frames 2>&1 | head -5 || true && \
         echo '' && echo '=== tf2_echo ldlidar_link base_footprint ===' && \
         timeout 3 ros2 run tf2_ros tf2_echo ldlidar_link base_footprint 2>&1 | head -10 || true"
    exit $?
fi

# ── Permisos X11 para RViz2 ───────────────────────────────────────
xhost +local:root 2>/dev/null || true

# ── Detectar display (X11 o Wayland) ─────────────────────────────
DISPLAY_ARGS=()
if [ -n "$DISPLAY" ]; then
    DISPLAY_ARGS+=(--env "DISPLAY=$DISPLAY")
    DISPLAY_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi
if [ -n "$WAYLAND_DISPLAY" ]; then
    DISPLAY_ARGS+=(--env "WAYLAND_DISPLAY=$WAYLAND_DISPLAY")
    DISPLAY_ARGS+=(-v "/run/user/$(id -u)/$WAYLAND_DISPLAY:/run/user/$(id -u)/$WAYLAND_DISPLAY:rw")
    DISPLAY_ARGS+=(--env "XDG_RUNTIME_DIR=/run/user/$(id -u)")
fi

# ── Detectar dispositivos conectados ────────────────────────────
DEVICE_ARGS=()
[ -e /dev/ttyUSB0 ] && DEVICE_ARGS+=(--device=/dev/ttyUSB0)
[ -e /dev/ttyUSB1 ] && DEVICE_ARGS+=(--device=/dev/ttyUSB1)
for v in /dev/video0 /dev/video1 /dev/video2 /dev/video3; do
    [ -e "$v" ] && DEVICE_ARGS+=(--device="$v")
done

# ── Construir imagen si no existe o si se pide rebuild ───────────
if ! podman image exists "$IMAGE" || [ "$1" = "rebuild" ]; then
    echo "━━━ Construyendo imagen Docker (primera vez, ~5 min) ━━━"
    podman build -t "$IMAGE" "$WORKSPACE"
    echo "━━━ Imagen construida ✅ ━━━"
fi

# ── Comando a ejecutar dentro del contenedor ─────────────────────
case "${1:-shell}" in

    build)
        CMD="source /opt/ros/jazzy/setup.bash && \
             cd /workspace && \
             echo '━━━ Instalando dependencias rosdep ━━━' && \
             rosdep install --from-paths src --ignore-src -r -y \
               --skip-keys 'ldlidar_component ldlidar_node ldlidar OrbbecSDK_ROS2 rescue_raspberry_brain' 2>/dev/null || true && \
             echo '━━━ Compilando workspace ━━━' && \
             colcon build --symlink-install \
               --packages-skip rescue_raspberry_brain \
               --packages-skip hector_mapping hector_geotiff hector_trajectory_server \
                   hector_imu_attitude_to_tf hector_imu_tools hector_compressed_map_transport \
                   hector_map_tools hector_marker_drawing hector_nav_msgs hector_slam \
                   world_info world_info_msgs rrl_launchers hector_compressed_map_transport \
               2>&1 && \
             echo '━━━ Build completo ✅ ━━━' && \
             source /workspace/install/setup.bash && \
             echo '━━━ Paquetes disponibles:' && \
             ros2 pkg list | grep rescue"
        ;;

    slam)
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Lanzando SLAM stack ━━━' && \
             ros2 launch rescue_bringup slam.launch.py"
        ;;

    lidar)
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Lanzando lidar LD19 ━━━' && \
             ros2 launch rescue_bringup lidar_ld19.launch.py"
        ;;

    scan)
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             ros2 topic echo /ldlidar_node/scan --once"
        ;;

    save-map)
        MAP="${2:-rescue_map}"
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             mkdir -p /root/maps && \
             ros2 run nav2_map_server map_saver_cli -f /root/maps/${MAP} && \
             echo 'Mapa guardado en ~/maps/${MAP}.yaml y ~/maps/${MAP}.pgm ✅'"
        ;;

    topics)
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             ros2 topic list"
        ;;

    rviz)
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             rviz2 -d /workspace/src/rescue_bringup/config/slam_rviz.rviz"
        ;;

    shell|*)
        CMD="bash"
        ;;
esac

echo "━━━ Dispositivos: ${DEVICE_ARGS[*]} ━━━"
echo "━━━ Display: ${DISPLAY_ARGS[*]} ━━━"

# ── Lanzar contenedor ─────────────────────────────────────────────
podman run -it --rm \
    --name "$CONTAINER" \
    --network host \
    --privileged \
    "${DEVICE_ARGS[@]}" \
    "${DISPLAY_ARGS[@]}" \
    --env "ROS_DOMAIN_ID=0" \
    --env "RCUTILS_COLORIZED_OUTPUT=1" \
    -v "$WORKSPACE:/workspace:z" \
    -v "/home/$(whoami)/maps:/root/maps:z" \
    "$IMAGE" \
    bash -c "$CMD"
