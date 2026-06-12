#!/usr/bin/env bash
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "Ejecuta este script con sudo:"
  echo "  sudo $0"
  exit 1
fi

TARGET_USER="${SUDO_USER:-${USER}}"

if [ -z "${TARGET_USER}" ] || [ "${TARGET_USER}" = "root" ]; then
  echo "No pude detectar el usuario normal. Usa:"
  echo "  sudo SUDO_USER=tu_usuario $0"
  exit 1
fi

echo "Habilitando permisos USB para el usuario: ${TARGET_USER}"

for group_name in dialout plugdev video input; do
  if getent group "${group_name}" >/dev/null; then
    usermod -aG "${group_name}" "${TARGET_USER}"
    echo "  OK: ${TARGET_USER} agregado a ${group_name}"
  else
    echo "  Aviso: el grupo ${group_name} no existe en esta maquina"
  fi
done

if command -v udevadm >/dev/null 2>&1; then
  udevadm control --reload-rules || true
  udevadm trigger || true
fi

echo
echo "Listo. Cierra sesion y vuelve a entrar, o reinicia la maquina."
echo "Verifica despues con:"
echo "  id"
echo "  ls -l /dev/ttyUSB* /dev/ttyACM* /dev/input/js* /dev/video*"
