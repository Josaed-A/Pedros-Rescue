#!/usr/bin/env bash
# Red ROS 2 — PC / estacion de mando.
# CycloneDDS unicast (sin multicast, sin discovery server).
# Autodetecta CABLE (eno1 -> Pi 10.42.0.240) o WIFI (wlp0s20f3 -> Pi 192.168.231.137).
#
# Uso:   source scripts/ros_net_pc.sh
#
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if ping -c1 -W1 10.42.0.240 >/dev/null 2>&1; then
    _IF=eno1;        _PEER=10.42.0.240;      _SELF=10.42.0.1;       _NET=cable
else
    _IF=wlp0s20f3;   _PEER=192.168.231.137;  _SELF=192.168.231.15;  _NET=wifi
fi
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${_IF}\" multicast=\"false\"/></Interfaces></General><Discovery><Peers><Peer Address=\"${_PEER}\"/><Peer Address=\"${_SELF}\"/></Peers></Discovery></Domain></CycloneDDS>"

source /home/semillero/ros2_jazzy/install/setup.bash 2>/dev/null
source /home/semillero/Pedros-Rescue/install/setup.bash 2>/dev/null
for _VENDOR_LIB in /home/semillero/ros2_jazzy/install/*/opt/*/lib; do
    if [ -d "${_VENDOR_LIB}" ]; then
        export LD_LIBRARY_PATH="${_VENDOR_LIB}:${LD_LIBRARY_PATH:-}"
    fi
done
ros2 daemon stop >/dev/null 2>&1
echo "[red PC] CycloneDDS por ${_NET} (${_IF}) -> Pi ${_PEER}"
