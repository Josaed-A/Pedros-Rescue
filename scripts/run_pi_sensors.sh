#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# run_pi_sensors.sh  — corre EN la Raspberry Pi 5
# Inicia lidar LD19 + cámara Orbbec Astra Pro en un contenedor.
#
# Uso (desde la Pi o via SSH):
#   ./scripts/run_pi_sensors.sh           → sensores completos (lidar + cámara)
#   ./scripts/run_pi_sensors.sh lidar     → solo lidar LD19
#   ./scripts/run_pi_sensors.sh camera    → solo cámara Orbbec
#   ./scripts/run_pi_sensors.sh stop      → detener contenedor
#   ./scripts/run_pi_sensors.sh logs      → ver logs en vivo
#   ./scripts/run_pi_sensors.sh build     → compilar workspace en contenedor
# ─────────────────────────────────────────────────────────────────

set -e

WORKSPACE="$HOME/pedros"
IMAGE="localhost/pedros-rescue-ros2:jazzy"
CONTAINER="pedros_sensors"

# ── Stop / logs ──────────────────────────────────────────────────
if [ "${1:-}" = "stop" ]; then
    echo "━━━ Deteniendo contenedor ${CONTAINER}... ━━━"
    podman stop "$CONTAINER" 2>/dev/null || true
    echo "━━━ Detenido ✅ ━━━"
    exit 0
fi

if [ "${1:-}" = "logs" ]; then
    podman logs -f "$CONTAINER"
    exit 0
fi

# ── Build dentro del contenedor ───────────────────────────────────
if [ "${1:-}" = "build" ]; then
    echo "━━━ Compilando workspace en contenedor Pi ━━━"
    podman run -it --rm \
        --name "${CONTAINER}_build" \
        -v "$WORKSPACE:/workspace:z" \
        "$IMAGE" \
        bash -c "
            source /opt/ros/jazzy/setup.bash
            cd /workspace
            echo '--- rosdep ---'
            rosdep install --from-paths src --ignore-src -r -y \
              --skip-keys 'ldlidar_component ldlidar_node ldlidar OrbbecSDK_ROS2 rescue_raspberry_brain orbbec_camera orbbec_camera_msgs orbbec_description rescue_command_station' 2>/dev/null || true
            echo '--- colcon build ---'
            colcon build --symlink-install \
              --packages-skip rescue_raspberry_brain rescue_command_station hector_mapping hector_geotiff hector_trajectory_server hector_imu_attitude_to_tf hector_imu_tools hector_compressed_map_transport hector_map_tools hector_marker_drawing hector_nav_msgs hector_slam world_info world_info_msgs rrl_launchers orbbec_camera orbbec_camera_msgs orbbec_description
            echo '━━━ Build completo ✅ ━━━'
        "
    exit 0
fi

# ── Verificar imagen ──────────────────────────────────────────────
if ! podman image exists "$IMAGE"; then
    echo "━━━ Imagen no encontrada. Construyendo (primera vez ~10 min)... ━━━"
    podman build -t "$IMAGE" -f "$WORKSPACE/Dockerfile.pi" "$WORKSPACE"
    echo "━━━ Imagen construida ✅ ━━━"
fi

# ── Dispositivos ──────────────────────────────────────────────────
DEVICE_ARGS=()
# Lidar LD19 via GPIO UART (dtoverlay=uart0-pi5)
[ -e /dev/ttyAMA0 ] && DEVICE_ARGS+=(--device=/dev/ttyAMA0)
# USB completo para Orbbec Astra Pro (protocolo propietario)
[ -e /dev/bus/usb ] && DEVICE_ARGS+=(-v /dev/bus/usb:/dev/bus/usb:rw)

# ── CycloneDDS: interfaz eth0 de la Pi, peer = PC ─────────────────
CYCLONE_XML="<CycloneDDS><Domain>\
<General><Interfaces>\
<NetworkInterface name=\"eth0\" multicast=\"false\"/>\
</Interfaces></General>\
<Discovery><Peers>\
<Peer Address=\"10.42.0.1\"/>\
<Peer Address=\"10.42.0.240\"/>\
</Peers></Discovery>\
</Domain></CycloneDDS>"

# ── Comando según modo ────────────────────────────────────────────
case "${1:-sensors}" in
    lidar)
        LAUNCH_ARGS="launch_camera:=false"
        ;;
    camera)
        LAUNCH_ARGS="launch_lidar:=false"
        ;;
    *)
        LAUNCH_ARGS=""
        ;;
esac

CMD="source /opt/ros/jazzy/setup.bash && \
     source /workspace/install/setup.bash && \
     echo '━━━ Sensores Pi: LD19 + Orbbec Astra Pro ━━━' && \
     ros2 launch rescue_bringup pi_sensors.launch.py ${LAUNCH_ARGS}"

echo "━━━ Dispositivos: ${DEVICE_ARGS[*]} ━━━"

# ── Lanzar contenedor ─────────────────────────────────────────────
podman run -it --rm --replace \
    --name "$CONTAINER" \
    --network host \
    --privileged \
    "${DEVICE_ARGS[@]}" \
    --env "ROS_DOMAIN_ID=0" \
    --env "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    --env "CYCLONEDDS_URI=${CYCLONE_XML}" \
    -v "$WORKSPACE:/workspace:z" \
    "$IMAGE" \
    bash -c "$CMD"
