<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/jw5XpIER_40?si=1Fc3-khvRHnYqO9C&mute=1&autoplay=1&loop=1&playlist=jw5XpIER_40" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

**Space Robotics Bench (SRB)** is a comprehensive collection of environments and tasks for robotics research in the challenging domain of space. It provides a unified framework for developing and validating autonomous systems under diverse extraterrestrial scenarios. At the same time, its design is flexible and extensible to accommodate a variety of development workflows and research directions beyond Earth.

## Key Features

- **Highly Parallelized Simulation via [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)**: SRB supports thousands of parallel simulation instances to accelerate workflows such as online learning, synthetic dataset generation, parameter tuning, and validation.

- **On-Demand Procedural Generation with [SimForge](https://github.com/AndrejOrsula/simforge)**: Automated procedural generation of simulation assets is leveraged to provide a unique scenario for each simulation instance, with the ultimate goal of developing autonomous systems that are both robust and adaptable to the unpredictable domain of space.

- **Extensive Domain Randomization**: All simulation instances can be further randomized to enhance the generalization of autonomous agents towards variable environment dynamics, visual appearance, illumination conditions, as well as sensor and actuation noise.

- **Compatibility with [Gymnasium API](https://gymnasium.farama.org)**: All tasks are compatible with a standardized API to ensure seamless integration with a broad ecosystem of libraries and frameworks for robot learning research.

- **Seamless Interface with [ROS 2](https://ros.org) & [Space ROS](https://space.ros.org)**: Simulation states, sensory outputs and actions of autonomous systems are available through ROS 2 middleware interface, enabling direct interoperability with the vast (Space) ROS ecosystem.

- **Abstract Architecture**: The architecture of SRB is designed to be modular and extensible, allowing for easy integration of new assets, robots, tasks and workflows

> 📑 If you have any questions or suggestions regarding this documentation, don't hesitate to reach out to us! More often than not, a lack of understanding is a result of poor documentation... Therefore, we are always looking to improve it!

## Table of Contents (available in the left sidebar)

#### Overview

1. [Environments](envs/index.md)
1. [Robots](robots/index.md)
1. [Integrations & Interfaces](integrations/index.md)

#### Getting Started

4. [System Requirements](getting_started/requirements.md)
1. [Installation](getting_started/install.md)
   - [Native](getting_started/install_native.md)
   - [Docker (Recommended)](getting_started/install_docker.md)
   - [Apptainer/Singularity](getting_started/install_apptainer.md)
1. [Basic Usage](getting_started/basic_usage.md)
1. [Workflows](workflows/index.md)
   - [ROS 2](workflows/ros2.md)
   - [Reinforcement Learning](workflows/reinforcement_learning.md)
   - [Sim-to-Real Transfer](workflows/sim_to_real.md)

#### Configuration

8. [Environment Configuration](config/env_cfg.md)
1. [Agent Configuration](config/agent_cfg.md)

#### Reference

10. [Command Line Interface (CLI)](reference/cli.md)
01. [Graphical User Interface (GUI)](reference/gui.md)

#### Development

12. [IDE Configuration](development/ide.md)
01. [Dev Container](development/devcontainer.md)
01. [Testing](development/testing.md)
01. [Documentation](development/documentation.md)
01. [Utilities](development/utilities.md)

#### Contributing

17. [New Assets](contributing/new_assets.md)
01. [New Tasks](contributing/new_tasks.md)

#### Miscellaneous

- [Attributions](misc/attributions.md)
- [Contributors](misc/contributors.md)
- [Citation](misc/citation.md)
- [Community](misc/community.md)
- [Troubleshooting](misc/troubleshooting.md)
