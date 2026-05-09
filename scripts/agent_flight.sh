#!/bin/bash
set -e

# Source ROS2 setup first
source /opt/ros/humble/setup.bash

# Source the workspace setup if it exists
if [ -f install/setup.bash ]; then 
    source install/setup.bash
fi

echo "Starting MicroXRCE-DDS Agent for Flight (serial)"

export PATH=$PATH:/usr/local/bin

if command -v MicroXRCEAgent &> /dev/null; then
    MicroXRCEAgent udp4 -p 8888
elif [ -f /usr/local/bin/MicroXRCEAgent ]; then
    /usr/local/bin/MicroXRCEAgent udp4 -p 8888
elif [ -f ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent ]; then
    ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888
else
    echo "Error: MicroXRCEAgent not found. Please install it first."
    echo "See ARCHITECTURE.md for setup instructions."
    exit 1
fi
