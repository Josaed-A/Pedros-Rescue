FROM docker.io/osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ── Dependencias del sistema ──────────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
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
    python3-opencv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── Ultralytics YOLO para detección de objetos ────────────────────
RUN pip3 install --no-cache-dir ultralytics==8.3.* 2>/dev/null || \
    pip3 install --no-cache-dir ultralytics

# ── Pre-descargar modelo YOLOv8n (~6 MB) para uso offline ────────
RUN python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" || \
    echo "Descarga de YOLOv8n diferida (sin internet en build)"

# ── Workspace ─────────────────────────────────────────────────────
WORKDIR /workspace

# ── Inicializar rosdep ────────────────────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ── Source ROS 2 en bashrc del root ──────────────────────────────
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /root/.bashrc

# ── Punto de entrada ─────────────────────────────────────────────
CMD ["/bin/bash"]
