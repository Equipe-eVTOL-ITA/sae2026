# sae2026

Competition repository for **SAE 2026** — eVTOL ITA.

## Structure

```
sae2026/
├── scripts/          ← build, simulate, agent scripts
│   ├── build.sh
│   ├── simulate.sh
│   └── agent.sh
├── mission_1/        ← ROS2 package for mission 1
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/
│   ├── src/
│   └── launch/
└── README.md
```

## Dependencies

This competition repo uses the following team packages (cloned side-by-side in `src/`):

| Package | What it provides |
|---|---|
| `fsm` | Finite state machine framework |
| `drone_lib` | PX4 drone abstraction (`Drone` class) |
| `stdstates` | Reusable states (takeoff, landing, PID, movement) |
| `cv_nodes` | Computer vision ROS2 nodes |
| `custom_msgs` | Shared message/service definitions |

See [ARCHITECTURE.md](../../../ARCHITECTURE.md) for the full workspace guide.

## Quick Start

```bash
# From the workspace root (evtol/dev/):
# 1. Build dependencies first
bash src/sae2026/scripts/build.sh deps

# 2. Build mission packages
bash src/sae2026/scripts/build.sh mission_1

# 3. Start simulation
bash src/sae2026/scripts/agent.sh      # Terminal 1
bash src/sae2026/scripts/simulate.sh <world>  # Terminal 2
source install/setup.bash && ros2 run mission_1 <node>  # Terminal 3
```

## License

MIT
