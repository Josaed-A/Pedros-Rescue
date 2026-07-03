FROM docker.io/osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ── Dependencias del sistema ──────────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-python \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-rclpy \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-joy \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-graph \
    libssl-dev \
    libudev-dev \
    libusb-1.0-0-dev \
    v4l-utils \
    usbutils \
    python3-pil \
    python3-numpy \
    ros-jazzy-backward-ros \
    ros-jazzy-image-publisher \
    ros-jazzy-camera-info-manager \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-camera-calibration-parsers \
    ros-jazzy-openni2-camera \
    ros-jazzy-v4l2-camera \
    ros-jazzy-depth-image-proc \
    ros-jazzy-depthimage-to-laserscan \
    ros-jazzy-image-proc \
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    libdw-dev \
    libopenni2-0 \
    libopenni2-dev \
    libuvc-dev \
    libeigen3-dev \
    libopencv-dev \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-geometry \
    ros-jazzy-image-transport \
    ros-jazzy-message-filters \
    ros-jazzy-rclcpp-components \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-class-loader \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-opencv \
    python3-tk \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── Ultralytics YOLO para detección de objetos ────────────────────
# --ignore-installed evita que pip intente desinstalar paquetes de apt (numpy, etc.)
# que no tienen RECORD file; --break-system-packages evita el bloqueo PEP 668
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed ultralytics==8.3.* 2>/dev/null || \
    pip3 install --no-cache-dir --break-system-packages --ignore-installed ultralytics

# ── zxing-cpp — detector QR (más resiliente que cv2.QRCodeDetector/pyzbar
# ante ángulo, distancia y códigos parcialmente cortados) ────────────
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed zxing-cpp

# ── pupil-apriltags — detector AprilTag tagStandard41h12 (cv2.aruco NO
# soporta esta familia) ───────────────────────────────────────────
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed pupil-apriltags

# ── CLIP — requerido por YOLO-World (set_classes) para detección de
# objetos de misión que no existen en COCO (rope, hard hat, etc.) ────
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    "git+https://github.com/ultralytics/CLIP.git"

# ── GUI del brazo 6-DOF (rescue_command_station/arm): customtkinter + matplotlib ─
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed customtkinter matplotlib

# ── Pre-descargar modelo YOLOv8n (~6 MB) para uso offline ────────
RUN python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" || \
    echo "Descarga de YOLOv8n diferida (sin internet en build)"

# ── Fijar setuptools compatible con colcon --symlink-install ─────
# ultralytics/customtkinter suben setuptools a una versión que elimina
# el flag --editable de "setup.py develop", rompiendo el build de
# paquetes ament_python (joy, rescue_interfaces, etc.)
RUN pip3 install --no-cache-dir --break-system-packages "setuptools==70.3.0"

# ── Workspace ─────────────────────────────────────────────────────
WORKDIR /workspace

# ── Inicializar rosdep ────────────────────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ── Source ROS 2 en bashrc del root ──────────────────────────────
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc

# ── Punto de entrada ─────────────────────────────────────────────
CMD ["/bin/bash"]
