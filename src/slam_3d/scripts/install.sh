#!/usr/bin/env bash
#
# install.sh -- dependencias del SLAM 3D (Orbbec Astra Pro + RTAB-Map)
#
# Correr en LAS DOS maquinas (Raspberry Pi y PC). El script detecta la
# arquitectura y solo instala lo que hace falta en cada una.
#
#   bash scripts/install.sh
#
set -e

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ARCH="$(uname -m)"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== Instalando dependencias para ROS 2 ${ROS_DISTRO} en ${ARCH} ==="

# ---------------------------------------------------------------------------
# 1. Paquetes ROS
# ---------------------------------------------------------------------------
# No se copian al repo: son de terceros, pesan cientos de MB y apt siempre
# trae la version correcta para tu distro.
sudo apt update
sudo apt install -y \
  "ros-${ROS_DISTRO}-rtabmap-ros" \
  "ros-${ROS_DISTRO}-openni2-camera" \
  "ros-${ROS_DISTRO}-v4l2-camera" \
  "ros-${ROS_DISTRO}-image-transport-plugins" \
  "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
  "ros-${ROS_DISTRO}-topic-tools" \
  v4l-utils

# ---------------------------------------------------------------------------
# 2. Reglas udev (permisos de la camara)
# ---------------------------------------------------------------------------
# Sin esto OpenNI2 encuentra el dispositivo pero no puede abrirlo:
#   "Could not open 2bc5/0403: Access denied (insufficient permissions)!"
echo "=== Instalando reglas udev ==="
sudo cp "${PKG_DIR}/config/99-astra.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "    Desconecta y reconecta la camara para que apliquen."

# ---------------------------------------------------------------------------
# 3. Driver de Orbbec
# ---------------------------------------------------------------------------
# El repo trae openni2_redist/ con liborbbec.so para arm64 y x64.
# Viene de https://github.com/orbbec/ros2_astra_camera (carpeta
# astra_camera/openni2_redist/). Se incluye porque la pagina oficial de Orbbec
# ya no lo ofrece de forma fiable y sin ese .so nada funciona.
case "${ARCH}" in
  aarch64) REDIST="${PKG_DIR}/openni2_redist/arm64" ;;
  x86_64)  REDIST="${PKG_DIR}/openni2_redist/x64"   ;;
  *) echo "Arquitectura no soportada: ${ARCH}"; exit 1 ;;
esac

if [ ! -f "${REDIST}/OpenNI2/Drivers/liborbbec.so" ]; then
  echo "!! No encuentro ${REDIST}/OpenNI2/Drivers/liborbbec.so"
  echo "!! Clonalo con:"
  echo "     git clone --depth 1 https://github.com/orbbec/ros2_astra_camera.git /tmp/astra"
  echo "     cp -r /tmp/astra/astra_camera/openni2_redist ${PKG_DIR}/"
  exit 1
fi
echo "=== Driver de Orbbec encontrado en ${REDIST} ==="

# ---------------------------------------------------------------------------
# 4. Buffers de socket (para DDS sobre red)
# ---------------------------------------------------------------------------
sudo sysctl -w net.core.rmem_max=16777216 > /dev/null
echo 'net.core.rmem_max=16777216' | sudo tee /etc/sysctl.d/60-ros2-dds.conf > /dev/null

echo
echo "=== Listo ==="
echo "Falta configurar el entorno ROS. Ver README.md, seccion 'Red'."
