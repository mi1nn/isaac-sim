# Interface — ROS 2 & Space ROS

All environments and workflows in the Space Robotics Bench support the ROS 2 interface, which allows you to communicate with each environment instance using standard ROS 2 middleware interfaces. In doing so, you can collect sensor data, control robots, and interact with the simulation instances using the vast ecosystem of ROS tools and libraries.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/Eohg5A6JgmE?si=RxobxeqbXkz_TWNi&mute=1&autoplay=1&loop=1&playlist=Eohg5A6JgmE" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Motivation

[ROS 2](https://ros.org) has become the de facto standard for developing robotic systems across various domains. The Space Robotics Bench provides a comprehensive ROS 2 interface that enables seamless integration between the simulation environment and the broader ROS ecosystem, offering several key benefits:

- **Compatibility with existing tools** - Leverage the rich ecosystem of ROS tools like RViz2, tf2, and rosbag2 for visualization, debugging, and data recording
- **Workflow continuity** - Develop algorithms and control systems that can transition smoothly from simulation to real hardware using the same interfaces
- **Distributed architecture** - Take advantage of ROS's node-based architecture to distribute computation and simulation across different processes or even machines
- **Community standardization** - Utilize standard message types (`geometry_msgs`, `sensor_msgs`, ...) that are widely understood and supported in the robotics community

### Space ROS

[Space ROS](https://space.ros.org) is an initiative aimed at extending the ROS ecosystem for applications beyond Earth. The ROS 2 interface of SRB is fully compatible with Space ROS, allowing you to develop and validate autonomous systems across a wide range of diverse extraterrestrial scenarios. In fact, the Space Robotics Bench is a spiritual continuation of the *Parallel ProcGen Environments* project developed during the [NASA Space ROS Sim Summer Sprint Challenge](https://www.freelancer.com/contest/NASA-Space-ROS-Sim-Summer-Sprint-Challenge-2417552/updates).

You can integrate SRB with Space ROS in two ways:

- **Direct**: Run the ROS 2 interface of SRB using the Space ROS middleware stack alongside the rest of your Space ROS nodes, which can be achieved simply by sourcing your Space ROS environment before running SRB
- **Indirect (recommended)**: Run the ROS 2 interface of SRB using the standard ROS 2 middleware stack and communicate with your Space ROS nodes using the standard ROS 2 interfaces (no additional setup required)

## Implementation

The ROS 2 interface is implemented through a dynamically configured bridge node with the following key features:

- **Dynamic ROS interfaces** - All publishers, subscribers, and services are created automatically based on the selected simulation environment without any manual configuration
- **Parallel environment support** - Every parallel simulation instance can be managed through a separate ROS namespace for seamless parallelization, while global control is also available
- **Standard message translation** - Environment states, sensory outputs, and control commands are automatically translated from simulation tensors to appropriate ROS interfaces
- **Complete access** - All simulation entities (robots, sensors, objects, ...) are automatically detected, and their interfaces are exposed to ROS alongside task-specific MDP rewards and signals

## How to Get Started?

If you want to start using the Space Robotics Bench with ROS 2, follow these steps:

1. [Installation](../getting_started/install.md)
1. [Basic Usage](../getting_started/basic_usage.md)
1. [ROS 2 Workflow](../workflows/ros2.md)
