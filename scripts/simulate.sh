#!/bin/bash
set -e

if [ -f install/setup.bash ]; then 
    source install/setup.bash
fi

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds

cd ~/PX4-Autopilot

PX4_SYS_AUTOSTART=4001
PX4_GZ_WORLD=$1

case $1 in
    sae1)
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500_sae
        ;;
    sae2)
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500_sae
        ;;
    sae3)
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500_sae
        ;;
    default)
        PX4_GZ_MODEL_POSE="0.0, 0.0, 0.05, 0.0, 0.0, 0.0"
        PX4_SIM_MODEL=x500
        ;;
    *)
        echo "Unknown world: $1"
        echo "Usage: $0 <world_name>"
        echo ""
        echo "Available worlds: sae1, sae2, sae3, default"
        exit 1
        ;;
esac

PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART \
PX4_GZ_MODEL_POSE=$PX4_GZ_MODEL_POSE \
PX4_GZ_WORLD=$PX4_GZ_WORLD \
PX4_SIM_MODEL=$PX4_SIM_MODEL \
./build/px4_sitl_default/bin/px4
