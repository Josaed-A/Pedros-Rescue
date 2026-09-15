#!/usr/bin/env bash
#
# setup_env.sh -- variables de entorno para que el PC y la Pi se vean
#
# Uso:
#   source scripts/setup_env.sh <IP_DE_LA_OTRA_MAQUINA> [INTERFAZ]
#
# Ejemplos:
#   # en la Pi (la otra maquina es el PC)
#   source scripts/setup_env.sh 10.230.234.238 wlan0
#   # en el PC (la otra maquina es la Pi)
#   source scripts/setup_env.sh 10.230.234.137 wlp1s0
#
# Para que sea permanente, agregalo al final de tu ~/.bashrc.

PEER_IP="$1"
IFACE="${2:-$(ip route get 1.1.1.1 2>/dev/null | grep -oP 'dev \K\S+' | head -1)}"

if [ -z "${PEER_IP}" ]; then
  echo "Uso: source setup_env.sh <IP_DE_LA_OTRA_MAQUINA> [INTERFAZ]"
  return 1 2>/dev/null || exit 1
fi

# Las dos maquinas tienen que usar el MISMO middleware.
# Fast DDS (el default) y Cyclone NO se hablan entre si: si una usa uno y la
# otra el otro, no se ven aunque la red este perfecta y el ping funcione.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# Descubrimiento por peers en vez de multicast.
# Muchas redes (WiFi institucional, hotspots de celular) bloquean el multicast
# UDP que ROS 2 usa por defecto para descubrir nodos. Sintoma tipico: SSH
# funciona, los dos estan en la misma subred, pero `ros2 topic list` desde el PC
# sale vacio. Apuntando cada lado al otro por IP el descubrimiento es unicast y
# deja de depender del multicast.
#
# FragmentSize importa: cada imagen de ~1 MB se parte en cientos de datagramas
# UDP y si uno se pierde se descarta el frame entero. Bajar el fragmento mejoro
# el throughput efectivo de 2 Hz a 8 Hz en pruebas sobre WiFi.
export CYCLONEDDS_URI="<CycloneDDS><Domain>\
<General>\
<Interfaces><NetworkInterface name=\"${IFACE}\"/></Interfaces>\
<MaxMessageSize>65500B</MaxMessageSize>\
<FragmentSize>4000B</FragmentSize>\
</General>\
<Discovery><Peers><Peer address=\"${PEER_IP}\"/></Peers></Discovery>\
</Domain></CycloneDDS>"

echo "ROS_DOMAIN_ID       = ${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION  = ${RMW_IMPLEMENTATION}"
echo "Interfaz            = ${IFACE}"
echo "Peer                = ${PEER_IP}"
