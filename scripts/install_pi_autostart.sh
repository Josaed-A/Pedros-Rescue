#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# install_pi_autostart.sh  —  corre UNA SOLA VEZ en la Raspberry Pi
#
# Instala un servicio systemd que arranca TODO el stack de sensores
# automaticamente al encender la Pi:
#   · Lidar LD19
#   · Camara Orbbec Astra Pro (depth + color)
#   · Camara Logitech frontal
#   · Detector de objetos (AprilTag + YOLO + hazmat)
#
# Uso (desde la Pi):
#   chmod +x ~/pedros/scripts/install_pi_autostart.sh
#   ~/pedros/scripts/install_pi_autostart.sh
#
# Para desinstalar:
#   systemctl --user disable --now pedros-sensors
#   rm ~/.config/systemd/user/pedros-sensors.service
# ─────────────────────────────────────────────────────────────────

set -e

WORKSPACE="$HOME/pedros"
SERVICE_DIR="$HOME/.config/systemd/user"
SERVICE_FILE="$SERVICE_DIR/pedros-sensors.service"

echo "━━━ Pedro Rescue — Instalando autostart en la Pi ━━━"

# ── 1. Habilitar linger (servicios de usuario sin sesion activa) ──
echo "[1/4] Habilitando linger para usuario $(whoami)..."
sudo loginctl enable-linger "$(whoami)" || {
    echo "AVISO: loginctl fallo — los servicios solo correran con sesion activa"
}

# ── 2. Crear directorio de servicios de usuario ───────────────────
echo "[2/4] Creando directorio de servicios..."
mkdir -p "$SERVICE_DIR"

# ── 3. Escribir archivo de servicio systemd ───────────────────────
echo "[3/4] Escribiendo servicio systemd..."
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Pedro Rescue - Stack de Sensores (lidar + camaras + detector)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=${WORKSPACE}
ExecStartPre=/bin/sleep 10
ExecStart=${WORKSPACE}/scripts/run_pi_sensors.sh
Restart=on-failure
RestartSec=20
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=default.target
EOF

echo "   Servicio escrito en: $SERVICE_FILE"

# ── 4. Habilitar y arrancar el servicio ───────────────────────────
echo "[4/4] Habilitando servicio..."
systemctl --user daemon-reload
systemctl --user enable pedros-sensors.service
systemctl --user start  pedros-sensors.service

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " Autostart instalado correctamente."
echo ""
echo " Al encender la Pi, el stack de sensores arranca solo."
echo " (espera ~15 s tras el boot para que los nodos esten listos)"
echo ""
echo " Comandos utiles:"
echo "   systemctl --user status  pedros-sensors   # ver estado"
echo "   journalctl --user -u pedros-sensors -f    # ver logs en vivo"
echo "   systemctl --user restart pedros-sensors   # reiniciar sensores"
echo "   systemctl --user stop    pedros-sensors   # detener sensores"
echo "   systemctl --user disable pedros-sensors   # desactivar autostart"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
