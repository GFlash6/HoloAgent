# HoloAgent Agentic Robot System

HoloAgent Agentic Robot System is a ROS 2 robotics stack for natural-language-driven robot skills, navigation, perception, semantic mapping, and robot-specific control adapters.

The repository currently supports two operating styles:

- **Agentic mode**: skills are registered and composed from natural-language tasks.
- **Workflow mode**: predefined scripts orchestrate known workflows for debugging.

This project targets real robot deployment environments. A hardware-free quick start and public model distribution workflow are still being prepared.

## Repository Layout

```text
agentic_robot_system/
├── agentic_robot/
│   ├── agentOS/
│   │   ├── holoagent_skills/ # Skill registry, per-skill docs, examples, and CRUD helpers
│   │   ├── run_dameon/       # Background daemon helpers
│   │   └── sandbox_test/     # Long-horizon and integration test scripts
│   ├── core/                 # Robot-agnostic ROS 2 workspace
│   ├── services/             # HTTP/ROS bridge and multi-robot control services
│   ├── thirdparty/           # Vendored ROS/C++ dependencies used by core
│   ├── fsr_vln/              # Semantic mapping and retrieval API
│   └── tools/                # Mapping and utility toolboxes
├── robots/
│   ├── unitree/              # Unitree robot-specific ROS 2 workspace and scripts
│   └── hexfellow/            # HexFellow-specific ROS 2 workspace and scripts
├── scripts/
│   ├── build.sh              # Workspace build helper
│   ├── container/            # Container lifecycle helpers
│   ├── intergation/          # Integration/workflow launch helpers
│   ├── perception/           # Perception launch helpers
│   └── recording/            # Recording helpers
├── README_agent.md           # Agentic mode runbook
└── README_workflow.md        # Workflow mode runbook
```



## Main Components

- `agentic_robot/core/src/nav_bringup`: Navigation2 launch files and parameters.
- `agentic_robot/core/src/navigation`: navigation executors and semantic/relative goal packages.
- `agentic_robot/core/src/perception`: perception ROS nodes and GPU inference integration.
- `agentic_robot/core/src/fast_livo`: FAST-LIVO-based mapping and relocalization integration.
- `agentic_robot/services/src/robot_bridge`: YAML-driven HTTP-to-ROS bridge.
- `agentic_robot/services/src/multi_robot_ctl`: multi-robot HTTP control examples.
- `robots/unitree/src`: Unitree robot arm and motion control packages.
- `robots/hexfellow/src`: HexFellow camera detection, lift control, and interfaces.



## Requirements

Expected baseline:

- Ubuntu with ROS 2 Humble-compatible tooling
- `colcon`, `rosdep`, and standard ROS 2 build tools
- Python 3 for ROS 2 Python packages and service utilities
- CUDA-capable GPU for perception and semantic mapping workflows
- Hardware-specific drivers for the selected robot, camera, IMU, LiDAR, and actuator stack

Important external dependencies include `livox_ros_driver2`, GTSAM, PCL, OpenCV, Eigen, `cv_bridge`, and `image_transport`. Some workflows also require ZED, Unitree, HexFellow, Qwen Model Studio, model weights, and local map/data assets.

Model weights, generated outputs, build artifacts, and local deployment configs are intentionally not tracked in this repository.

## ROS Humble Docker Dependency Setup

The currently tested setup starts from a ROS 2 Humble Docker image, then installs the system and driver dependencies below before building this repository.

```bash
apt update

# Sophus
apt install -y ros-humble-sophus

# GTSAM
apt install -y cmake libboost-all-dev libtbb-dev
apt install -y ros-humble-gtsam

# PCL
apt install -y libpcl-dev ros-humble-pcl-conversions ros-humble-pcl-msgs

# OpenCV and ROS image bridges
apt install -y libopencv-dev ros-humble-cv-bridge ros-humble-image-transport

# Navigation2 dependencies used by the vendored thirdparty workspace
apt install -y \
  ros-humble-bondcpp ros-humble-test-msgs \
  ros-humble-behaviortree-cpp-v3 ros-humble-diagnostic-updater \
  libgraphicsmagick++1-dev ros-humble-rviz2 ros-humble-angles \
  libunwind-dev libgoogle-glog-dev libceres-dev \
  libxtensor-dev libxsimd-dev libompl-dev libnanoflann-dev

apt install -y ros-humble-tf2-* ros-humble-tf-transformations
apt install -y --only-upgrade ros-humble-geometry-msgs

# Common tools
apt install -y tmux git wget curl python3-pip
```



## Build

Build the main workspace:

```bash
bash scripts/build.sh
```

Build a specific package group through the helper:

```bash
bash agentic_robot/build.sh -p robot_bridge
```

`agentic_robot/core` is not a fully standalone workspace. It depends on packages that are currently vendored under `agentic_robot/thirdparty`, including Navigation2 packages such as `nav2_simple_commander` and `nav2_bringup`, and `rpg_vikit-ros2` packages such as `vikit_common` and `vikit_ros`.

The normal build order is split by layer:

1. Prepare ROS/system/driver dependencies.
2. Build the non-robot `agentic_robot` layer: `thirdparty` -> `core` -> `services`.
3. Build one robot-specific workspace: `robots/unitree` or `robots/hexfellow`.

Build the non-robot layer first:

```bash
bash agentic_robot/build.sh --workspace all
```

Build a narrower non-robot target when working on one workspace:

```bash
bash agentic_robot/build.sh --workspace thirdparty
bash agentic_robot/build.sh --workspace core
bash agentic_robot/build.sh --workspace services
```

Then build the selected robot workspace:

```bash
bash robots/unitree/build.sh
# or
bash robots/hexfellow/build.sh
```

Build one non-robot package:

```bash
bash agentic_robot/build.sh --package nav_executor
bash agentic_robot/build.sh --workspace core --package perception
```

Build one robot package:

```bash
bash robots/unitree/build.sh --package g1_move
bash robots/hexfellow/build.sh --package <pkg>
```

Limit parallelism on small machines:

```bash
bash agentic_robot/build.sh --workspace core --jobs 2
bash robots/unitree/build.sh --jobs 2
```

The root-level `scripts/build.sh` remains as a compatibility dispatcher for older commands such as `bash scripts/build.sh --workspace core` or `bash scripts/build.sh --workspace unitree`. Prefer the layer-specific scripts for new workflows because they keep robot-independent dependencies separate from robot-specific dependencies.

After building, source workspaces from generic to specific:

```bash
source agentic_robot/thirdparty/install/setup.bash
source agentic_robot/core/install/setup.bash
source agentic_robot/services/install/setup.bash      # if robot_bridge was built
source robots/unitree/install/setup.bash              # or robots/hexfellow/install/setup.bash
```

You may use system ROS packages instead of vendored thirdparty packages, but avoid mixing both sources for the same dependency set in one environment.

## Running

Detailed operating procedures are still in the existing runbooks:

- Agentic mode: `[README_agent.md](README_agent.md)`
- Workflow mode: `[README_workflow.md](README_workflow.md)`

Repository launch/orchestration helpers are mainly under `scripts/container/`, `scripts/intergation/`, `scripts/perception/`, and `scripts/audio/`. These scripts assume a prepared robot/container environment and hardware-specific services; they are not a hardware-free quickstart.

## Configuration

Runtime-specific values should be provided through environment variables or local config files, not committed defaults. Common variables include:

- `ROBOT_ID`
- `CONTROL_URL`
- `ROBOT_11_URL` through `ROBOT_16_URL`
- `EXPECTED_ROBOTS`
- `QWEN_API_KEY`
- `QWEN_MODEL` (optional, defaults to `qwen3.7-plus`)
- `QWEN_BASE_URL` (optional)
- `HOLOAGENT_DATA_ROOT`

If chatbot voice interaction is required, also configure:

- `QWEN_API_KEY`
- `QWEN_ASR_MODEL` (optional, defaults to `qwen3-asr-flash-realtime`)
- `QWEN_ASR_URL` (optional)
- `CHATBOT_TTS_APP_KEY`
- `CHATBOT_TTS_ACCESS_KEY`

Local model paths, map paths, robot IPs, and service URLs should be treated as deployment configuration.

## Models And Data

Model files and datasets are not stored in this repository. Before running perception or semantic mapping, provide the required assets in the paths expected by the relevant config files, or override those paths through local configuration.

The model distribution plan, checksums, and license details still need to be finalized before public release.

## Entry Points

- Agentic workflow: see `[README_agent.md](README_agent.md)`
- Pre-defined workflow: see `[README_workflow.md](README_workflow.md)`
- Skill system: see `[agentic_robot/agentOS/holoagent_skills/README.md](agentic_robot/agentOS/holoagent_skills/README.md)`
- AgentOS overview: see `[agentic_robot/agentOS/README.md](agentic_robot/agentOS/README.md)`



## Third-Party Code

This repository contains vendored third-party source code. See `[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)` for the current notice list and open license-review items.

Known follow-up: resolve the FAST-LIVO license discrepancy between its README and package metadata before a public release.

## Safety

This software can command real robots and actuators. Run only in controlled environments with appropriate emergency-stop procedures, speed limits, and human supervision. LLM, vision, and semantic outputs must not be treated as safety-critical decisions.

## License

Repository-owned code is provided under the Apache License 2.0 unless a file, package, or third-party directory states otherwise. See `[LICENSE](LICENSE)` and `[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)`.

## Notes

- Many workflows assume robot-side services, maps, and model assets already exist.
- Third-party directories are vendored and should be treated separately from project-maintained documentation.
- Some scripts are tightly coupled to internal deployment topology and robot IP planning.

## Related Foundation Models

Some HoloAgent demos rely on foundation models from HorizonRobotics. Two directly related open-source projects are listed below.

### HoloBrain

[HoloBrain](https://horizonrobotics.github.io/robot_lab/holobrain/) is a foundation model for general embodied manipulation. It is used in the HexFellow mobile manipulation demos and supports heterogeneous robots through explicit embodiment priors, including camera parameters and kinematic descriptions. See the [open-source implementation](https://github.com/HorizonRobotics/RoboOrchardLab/tree/master/projects/holobrain) for more information.

### HoloMotion

[HoloMotion](https://horizonrobotics.github.io/robot_lab/holomotion/) is a foundation model for whole-body humanoid control. It is used in G1-related demos for robust whole-body motion tracking and provides an end-to-end workflow covering motion data, training, evaluation, and real-robot deployment. See the [open-source repository](https://github.com/HorizonRobotics/HoloMotion) for more information.
