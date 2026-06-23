<div align="center">
<img src="docs/assets/holoagent_logo_text.png" alt="HoloAgent Logo" width="420"/>

[![Project](https://img.shields.io/badge/📖-Project-blue)](https://horizonrobotics.github.io/robot_lab/holoagent/)
[![📄 arXiv](https://img.shields.io/badge/📄-arXiv-b31b1b)](https://arxiv.org/abs/2606.23565)
<!-- [![Code](https://img.shields.io/badge/Code-Coming%20Soon-lightgrey)](#release-status) -->
</div>

HoloAgent is a unified embodied-agent framework for general-purpose robots, integrating closed-loop execution, 3D spatial memory, and robot skills for real-world tasks.

## 🔥 News
- **[2026.06]** HoloAgent-0 is released. Code is under preparation and will be released soon.
- **[2025.09]** FSR-VLN is released for fast-and-slow vision-language navigation.

<!-- ## 🚀 HoloAgent-0
> ***HoloAgent-0*** connects Embodied AgentOS, 3D spatial memory, and embodied skills for real-world robot deployment.

It targets long-horizon navigation, object search, cross-robot coordination, mobile manipulation, and runtime-feedback-driven recovery. Code will be released in this repository after it is ready. -->

## ✅ Release Status
- [ ] HoloAgent-0 code update
- [x] HoloAgent-0 project page & paper
- [x] FSR-VLN code

## 🧩 Components
- **Embodied AgentOS:** Coordinates high-level task planning, runtime feedback, and closed-loop robot execution.
- **3D Spatial Memory:** Grounds robot reasoning in physical-world spatial representations for long-horizon tasks.
- **Embodied Skills:** Connects agent decisions to executable robot navigation and manipulation skills.
- **FSR-VLN:** Provides fast-and-slow vision-language navigation with a hierarchical multi-modal scene graph.

## 🧠 Framework for Closed-Loop Robot Execution

AgentOS turns language instructions into monitored skill graphs and closes the loop across spatial retrieval, execution, memory updates, and recovery.

<img src="docs/assets/holoagent_framework.png" alt="Overview of the HoloAgent-0 framework" width="900"/>

## 🤖 Real-Robot Demonstrations

Compressed previews from real-hardware deployments. Full-resolution videos are available on the project page.

| Navigation and Dance Coordination | Long-Horizon Mobile Manipulation |
|:---:|:---:|
| <img src="docs/assets/demos/navigation_dance.gif" alt="Navigation and dance coordination" width="420"/> | <img src="docs/assets/demos/mobile_manipulation.gif" alt="Long-horizon mobile manipulation" width="420"/> |
| Coordinate navigation and humanoid motion across robots. | Decompose long-horizon manipulation into navigation, grasping, placement, and recovery. |

| Active Exploration in a New Environment | Interactive Humanoid Command Execution |
|:---:|:---:|
| <img src="docs/assets/demos/active_exploration.gif" alt="Active exploration in a new environment" width="420"/> | <img src="docs/assets/demos/humanoid_command.gif" alt="Interactive humanoid command execution" width="420"/> |
| Explore new spaces and update 3D memory online. | Follow open-ended commands with navigation and embodied actions. |

| A Day with a Robot Companion | A Day in the Life of a Robot Guide |
|:---:|:---:|
| <img src="docs/assets/demos/robot_companion.gif" alt="A day with a robot companion" width="420"/> | <img src="docs/assets/demos/robot_guide.gif" alt="A day in the life of a robot guide" width="420"/> |
| Combine language, 3D reasoning, navigation, interaction, and action. | Guide users through workspaces with spatial-memory-aware routes. |

## 🤖 FSR-VLN
[![Project](https://img.shields.io/badge/📖-Project-blue)](https://horizonrobotics.github.io/robot_lab/fsr-vln)
[![📄 arXiv](https://img.shields.io/badge/📄-arXiv-b31b1b)](https://arxiv.org/abs/2509.13733)
[![中文介绍](https://img.shields.io/badge/中文介绍-07C160?logo=wechat&logoColor=white)](https://mp.weixin.qq.com/s/HqnBlTNqOL3Z4Kg8tLHCSw)
> ***FSR-VLN*** is the HoloAgent navigation component, combining a Hierarchical Multi-modal Scene Graph with Fast-to-Slow Navigation Reasoning for efficient long-range spatial reasoning.

<img src="docs/assets/FSR_VLN_framework.png" alt="Overall Framework" width="700"/>


## 🏗 Getting Started

The current repository includes FSR-VLN and navigation-agent setup. HoloAgent-0 code will be added in a future release.

### 1. Semantic Mapping and Retrieval Pipeline
- **Task:** Implement the semantic mapping and retrieval system based on the instructions in `fsr_vln/README.md`.
- **Steps:**
    1.  Download the necessary pre-trained model checkpoints.
    2.  Download and configure the required datasets.
    3.  Set up the environment and dependencies as specified.
    4.  Run the complete pipeline to verify its functionality for semantic mapping and visual place retrieval.

### 2. Navigation Agent Setup and Execution
- **Task:** Set up and test the navigation agent according to `nav_agent/README.md`.
- **Steps:**
    1.  Install all required dependencies for the navigation environment.
    2.  Configure the necessary parameters and environment settings.
    3.  Execute the navigation agent to ensure it runs successfully and performs its intended tasks.


## 📚 Publications & Citation

If you find our project useful, please consider citing it:

```bibtex
@misc{holoagent2026holoagent0,
      title={HoloAgent-0: A Unified Embodied Agent Framework with 3D Spatial Memory},
      year={2026},
      eprint={2606.23565},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2606.23565},
}
```

```bibtex
@misc{zhou2025fsrvlnfastslowreasoning,
      title={FSR-VLN: Fast and Slow Reasoning for Vision-Language Navigation with Hierarchical Multi-modal Scene Graph}, 
      author={Xiaolin Zhou and Tingyang Xiao and Liu Liu and Yucheng Wang and Maiyue Chen and Xinrui Meng and Xinjie Wang and Wei Feng and Wei Sui and Zhizhong Su},
      year={2025},
      eprint={2509.13733},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2509.13733}, 
}
```

## 🙏 Acknowledgements

This project is built upon and inspired by several outstanding open source projects: [OVO](https://github.com/tberriel/OVO)、[HOV-SG](https://github.com/hovsg/HOV-SG)、[rerun](https://github.com/rerun-io/rerun)、[dimos](https://github.com/dimensionalOS/dimos.git)、[openclaw](https://github.com/openclaw/openclaw).

---

## ⚖️ License

This project is licensed under the [Apache License 2.0](LICENSE). See the `LICENSE` file for details.
