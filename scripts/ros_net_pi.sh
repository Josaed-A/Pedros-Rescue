#!/usr/bin/env bash
# Red ROS 2 — Raspberry Pi (gardian).
# CycloneDDS unicast (sin multicast, sin discovery server).
# Autodetecta CABLE (eth0 -> PC 10.42.0.1) o WIFI (wlan0 -> PC 192.168.231.15).
#
# Uso (en la Pi):   source ~/pedros/scripts/ros_net_pi.sh
#
unset ROS_DISCOVERY_SERVER
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if ping -c1 -W1 10.42.0.1 >/dev/null 2>&1; then
    _IF=eth0;    _PEER=10.42.0.1;        _SELF=10.42.0.240;     _NET=cable
else
    _IF=wlan0;   _PEER=192.168.231.15;   _SELF=192.168.231.137; _NET=wifi
fi
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${_IF}\" multicast=\"false\"/></Interfaces></General><Discovery><Peers><Peer Address=\"${_PEER}\"/><Peer Address=\"${_SELF}\"/></Peers></Discovery></Domain></CycloneDDS>"

source /opt/ros/jazzy/setup.bash 2>/dev/null
source ~/pedros/install/setup.bash 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
echo "[red Pi] CycloneDDS por ${_NET} (${_IF}) -> PC ${_PEER}"
