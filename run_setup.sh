#!/bin/bash



# Temporary
# sudo sysctl -w net.core.rmem_max=2147483647
# sudo sysctl -w net.core.rmem_default=67108864

# ------
# Permanent
# echo "net.core.rmem_max=2147483647" | sudo tee -a /etc/sysctl.conf
# echo "net.core.rmem_default=67108864" | sudo tee -a /etc/sysctl.conf



echo "Current kernel UDP receive buffer:"
sysctl net.core.rmem_max
sysctl net.core.rmem_default
echo ""


echo "Deactivated conda"
conda deactivate 2>/dev/null || true
echo ""



echo "Setup unitree ros2 environment"
ROOT="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp2s0" priority="default" multicast="default" /></Interfaces><AllowMulticast>spdp</AllowMulticast></General><Internal><SocketReceiveBufferSize min="64MB"/></Internal></Domain></CycloneDDS>'
