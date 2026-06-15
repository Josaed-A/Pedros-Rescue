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
# Nombre de contenedor según modo para poder correr SLAM + dashboard en paralelo
case "${1:-shell}" in
    dashboard) CONTAINER="pedros_dashboard" ;;
    *) CONTAINER="pedros_slam" ;;
esac

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

# ── Modo save-geotiff: llama al servicio /save_geotiff ───────────
# Requiere que el contenedor ya esté corriendo con el SLAM stack.
# Uso: ./scripts/run_slam_container.sh save-geotiff
if [ "${1:-}" = "save-geotiff" ]; then
    podman exec -it "$CONTAINER" bash -c \
        "source /opt/ros/jazzy/setup.bash && \
         source /workspace/install/setup.bash 2>/dev/null || true && \
         echo '━━━ Guardando mapa GeoTIFF... ━━━' && \
         ros2 service call /save_geotiff std_srvs/srv/Trigger '{}'"
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
# Bus USB completo para Orbbec Astra Pro (depth sensor protocolo propietario)
[ -e /dev/bus/usb ] && DEVICE_ARGS+=(-v /dev/bus/usb:/dev/bus/usb:rw)

# ── Construir imagen si no existe o si se pide rebuild ───────────
if [ "$1" = "rebuild-pi" ]; then
    echo "━━━ Construyendo imagen Pi (sin RViz/YOLO, ~8 min) ━━━"
    podman build -t "$IMAGE" -f "$WORKSPACE/Dockerfile.pi" "$WORKSPACE"
    echo "━━━ Imagen Pi construida ✅ ━━━"
    exit 0
fi
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
               --packages-skip rescue_raspberry_brain hector_mapping hector_geotiff hector_trajectory_server hector_imu_attitude_to_tf hector_imu_tools hector_compressed_map_transport hector_map_tools hector_marker_drawing hector_nav_msgs hector_slam world_info world_info_msgs rrl_launchers orbbec_camera orbbec_camera_msgs orbbec_description \
               2>&1 && \
             echo '━━━ Build completo ✅ ━━━' && \
             source /workspace/install/setup.bash && \
             echo '━━━ Paquetes disponibles:' && \
             ros2 pkg list | grep rescue"
        ;;

    slam)
        # PC-mode: lidar y cámaras ya corren en Pi — solo SLAM + RViz
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Lanzando SLAM stack (PC) ━━━' && \
             ros2 launch rescue_bringup slam.launch.py \
               launch_lidar:=false launch_camera:=false"
        ;;

    camera-pi)
        # Publica la cámara Logitech en /robot/camera/front/image_raw/compressed
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Publicando Logitech /dev/video0 ━━━' && \
             ros2 run rescue_bringup logitech_pub --ros-args \
               -p device:=0 -p fps:=15 -p jpeg_quality:=75"
        ;;

    logitech-vision)
        # Logitech + object_detector (sin depth: detecta pero no localiza en 3D)
        # Uso: ./scripts/run_slam_container.sh logitech-vision [device=2]
        CAM_DEVICE="${2:-2}"
        HAZMAT_MODEL="/workspace/src/rescue_bringup/models/hazmat_yolo.pt"
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Logitech Vision (device=${CAM_DEVICE}) ━━━' && \
             ros2 launch rescue_bringup logitech_vision.launch.py \
               device:=${CAM_DEVICE} \
               hazmat_model:=${HAZMAT_MODEL}"
        ;;

    vision-test)
        # Prueba el detector de hazmat con la Logitech — sin SLAM
        # Uso: ./scripts/run_slam_container.sh vision-test [device=2]
        CAM_DEVICE="${2:-2}"
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Prueba detector hazmat (Logitech /dev/video${CAM_DEVICE}) ━━━' && \
             python3 /workspace/training/test_hazmat_camera.py --device ${CAM_DEVICE}"
        ;;

    vision)
        # Cámara Orbbec + object_detector completo (AprilTag + hazmat YOLO + objetos)
        HAZMAT_MODEL="/workspace/src/rescue_bringup/models/hazmat_yolo.pt"
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Lanzando módulo de visión ━━━' && \
             ros2 launch rescue_bringup vision.launch.py \
               hazmat_model:=${HAZMAT_MODEL} \
               launch_rviz:=true"
        ;;

    dashboard)
        # Ground station — dashboard Tkinter en el PC
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Lanzando Ground Station Dashboard ━━━' && \
             ros2 launch rescue_command_station command_station.launch.py"
        ;;

    slam-pi)
        # Modo headless para Raspberry Pi — sin RViz
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             export LD_LIBRARY_PATH=/workspace/install/astra_camera/lib:\$LD_LIBRARY_PATH && \
             export OPENNI2_REDIST=/workspace/install/astra_camera/lib && \
             export OPENNI2_DRIVERS_PATH=/workspace/install/astra_camera/lib/OpenNI2/Drivers && \
             echo '━━━ Lanzando SLAM headless (Pi) ━━━' && \
             ros2 launch rescue_bringup slam.launch.py launch_rviz:=false"
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
             source /workspace/install/setup.bash 2>/dev/null || true && \
             MESA_GL_VERSION_OVERRIDE=3.3 LIBGL_ALWAYS_SOFTWARE=0 \
             rviz2 -d /workspace/src/rescue_bringup/config/slam_rviz.rviz"
        ;;

    save-ply)
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Guardando nube de puntos 3D PLY... ━━━' && \
             ros2 service call /save_pointcloud_ply std_srvs/srv/Trigger '{}'"
        ;;

    save-csv)
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Guardando CSV de detecciones... ━━━' && \
             ros2 service call /save_detection_csv std_srvs/srv/Trigger '{}'"
        ;;

    save-mission)
        CMD="source /opt/ros/jazzy/setup.bash && \
             source /workspace/install/setup.bash && \
             echo '━━━ Guardando misión completa (GeoTIFF + PLY + CSV)... ━━━' && \
             ros2 service call /save_geotiff std_srvs/srv/Trigger '{}' && \
             echo '--- GeoTIFF listo ---' && \
             ros2 service call /save_pointcloud_ply std_srvs/srv/Trigger '{}' && \
             echo '--- PLY listo ---' && \
             ros2 service call /save_detection_csv std_srvs/srv/Trigger '{}' && \
             echo '━━━ Misión guardada: GeoTIFF + PLY + CSV ✅ ━━━' && \
             ls -lh /root/maps/ | tail -10"
        ;;

    # ── Modos que se ejecutan en la Pi via SSH ─────────────────────
    pi|pi-sensors|pi-lidar|pi-camera|pi-stop|pi-logs|pi-build)
        PI_HOST="sraus@10.42.0.240"
        PI_PASS="123456"
        PI_MODE="${1#pi}"          # "" | "-sensors" | "-lidar" | "-camera" | "-stop" | "-logs" | "-build"
        PI_ARG="${PI_MODE#-}"      # "" | "sensors" | "lidar" | "camera" | "stop" | "logs" | "build"
        [ -z "$PI_ARG" ] && PI_ARG="sensors"

        echo "━━━ Sincronizando scripts + launch a Pi... ━━━"
        sshpass -p "$PI_PASS" scp \
            "$WORKSPACE/scripts/run_pi_sensors.sh" \
            "${PI_HOST}:~/pedros/scripts/run_pi_sensors.sh" 2>/dev/null || true
        sshpass -p "$PI_PASS" scp \
            "$WORKSPACE/scripts/install_pi_autostart.sh" \
            "${PI_HOST}:~/pedros/scripts/install_pi_autostart.sh" 2>/dev/null || true
        sshpass -p "$PI_PASS" scp \
            "$WORKSPACE/src/rescue_bringup/launch/pi_sensors.launch.py" \
            "${PI_HOST}:~/pedros/src/rescue_bringup/launch/pi_sensors.launch.py" 2>/dev/null || true
        sshpass -p "$PI_PASS" scp \
            "$WORKSPACE/src/rescue_bringup/launch/logitech_vision.launch.py" \
            "${PI_HOST}:~/pedros/src/rescue_bringup/launch/logitech_vision.launch.py" 2>/dev/null || true

        echo "━━━ Lanzando sensores en Pi (modo: ${PI_ARG}) ━━━"
        sshpass -p "$PI_PASS" ssh -o StrictHostKeyChecking=no "$PI_HOST" \
            "chmod +x ~/pedros/scripts/run_pi_sensors.sh && \
             ~/pedros/scripts/run_pi_sensors.sh ${PI_ARG}"
        exit 0
        ;;

    shell|*)
        CMD="bash"
        ;;
esac

echo "━━━ Dispositivos: ${DEVICE_ARGS[*]} ━━━"
echo "━━━ Display: ${DISPLAY_ARGS[*]} ━━━"

# ── CycloneDDS: forzar interfaz enp3s0 (Pi via ethernet) ─────────
CYCLONE_XML="<CycloneDDS><Domain>\
<General><Interfaces>\
<NetworkInterface name=\"enp3s0\" multicast=\"false\"/>\
</Interfaces></General>\
<Discovery><Peers>\
<Peer Address=\"10.42.0.240\"/>\
<Peer Address=\"10.42.0.1\"/>\
</Peers></Discovery>\
</Domain></CycloneDDS>"

# ── Podman socket (solo para dashboard — permite lanzar rviz2 en pedros_slam) ──
PODMAN_SOCK_ARGS=()
if [ "${1:-}" = "dashboard" ]; then
    PDMN_SOCK="/run/user/$(id -u)/podman/podman.sock"
    systemctl --user start podman.socket 2>/dev/null || true
    # Esperar hasta 3 s a que el socket aparezca
    for _i in 1 2 3; do
        [ -S "$PDMN_SOCK" ] && break
        sleep 1
    done
    if [ -S "$PDMN_SOCK" ]; then
        PODMAN_SOCK_ARGS=(-v "${PDMN_SOCK}:/tmp/podman.sock:rw")
        echo "━━━ Podman socket montado para control de RViz ━━━"
    else
        echo "━━━ AVISO: no se pudo montar podman socket; boton RVIZ no funcionara ━━━"
    fi
fi

# ── Lanzar contenedor ─────────────────────────────────────────────
podman run -it --rm --replace \
    --name "$CONTAINER" \
    --network host \
    --privileged \
    "${DEVICE_ARGS[@]}" \
    "${DISPLAY_ARGS[@]}" \
    "${PODMAN_SOCK_ARGS[@]}" \
    --env "ROS_DOMAIN_ID=0" \
    --env "RCUTILS_COLORIZED_OUTPUT=1" \
    --env "CYCLONEDDS_URI=${CYCLONE_XML}" \
    -v "$WORKSPACE:/workspace:z" \
    -v "/home/$(whoami)/maps:/root/maps:z" \
    "$IMAGE" \
    bash -c "$CMD"
