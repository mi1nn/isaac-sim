# Sim-to-Real Transfer

The Space Robotics Bench provides a streamlined workflow for deploying agents trained in simulation directly onto physical hardware. This is managed through the `real_agent` command-line interface, which runs a hardware-interfacing equivalent of a simulated environment. This allows various workflows, including the deployment of a trained RL policy, to be executed on a real robot with minimal changes.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/BGAxdTRTG80?si=ub6p2hDG9bnTAJSK&mute=1&autoplay=1&loop=1&playlist=BGAxdTRTG80" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 1. Train your Agent in Simulation

> Reference: [Reinforcement Learning Workflow](./reinforcement_learning.md)

The first step is to train a policy in simulation. The goal is to produce a stable policy checkpoint. **All RL frameworks and algorithms integrated into SRB are supported by this sim-to-real workflow.**

Let's train a Dreamer agent for the `waypoint_navigation` task with the Leo Rover on 512 parallel environments. Since we are deploying in an on-Earth facility, we will also specify the `earth` gravity setting. (For the best results, it is highly recommended to use a combination of Domain Randomization and Procedural Generation over the course of the training. This creates a more robust agent that is better prepared for the complexities of the real world.)

```bash
srb agent train --headless --algo dreamer --env waypoint_navigation env.robot=leo_rover env.num_envs=512 env.domain=earth
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/ot43Q3tuIv4?si=wNcYdfcmGBPscqRD&mute=1&autoplay=1&loop=1&playlist=ot43Q3tuIv4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 2. Generate Sim-to-Real Bridge

> Reference: [`srb real_agent gen` — Generate Sim-to-Real Bridge](../reference/cli_real_agent_gen.md)

This key step creates the bridge between the simulation and the real world. The `srb real_agent gen` command inspects a simulated Gymnasium environment and automatically writes a lightweight, real-world counterpart that does not depend on the simulation backend.

You can specify which default `HardwareInterface` modules your robot uses via the `--hardware` flag. These are the drivers that communicate with your robot's software, e.g., via ROS 2. For the Leo Rover, we will need an interface to send velocity commands (`ros_cmd_vel`) and one to receive pose information (`ros_tf`). Furthermore, we will use the `ros_mw` interface to expose ROS 2 service calls for pausing and resuming the agent. It is important to specify the robot here so that its parameters, such as action scaling, can be correctly extracted for the real-world environment.

```bash
srb real_agent gen --env waypoint_navigation env.robot=leo_rover --hardware ros_cmd_vel ros_tf ros_mw
```

This command launches a temporary headless SRB session. It loads the environment, inspects its APIs, and then writes a new Python file inside the [`sim_to_real/env`](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/srb/interfaces/sim_to_real/env) directory. This Python file defines a `RealEnv` class and registers it in Gymnasium under the `srb_real/` namespace.

## 3. Deploy and Evaluate on Hardware

With the bridge generated, you can deploy the agent to your robot. The `real_agent` command does **not** launch a simulation. Instead, it runs the generated `RealEnv`, which connects directly to your hardware.

To evaluate the policy trained in the first step, run the following command.

```bash
srb real_agent eval --env waypoint_navigation --algo dreamer
```

The `RealEnv` will instantiate the `ros_cmd_vel`, `ros_tf`, and `ros_mw` interfaces. When the policy produces an action, the environment routes it to the `RosCmdVelInterface`, which publishes it as a ROS 2 message. It then gets the latest pose from the `RosTfInterface` to use as an observation for the policy's next step.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/inHHDDrnjrw?si=XRYcYqmrOQqla9jg&mute=1&autoplay=1&loop=1&playlist=inHHDDrnjrw" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 4. Advanced Workflows and Use Cases

The `real_agent` tool is not just for evaluation. It enables several powerful workflows for research and development.

### Debugging with Zero and Random Agents

Before deploying a fully autonomous policy, you can use the `zero` and `random` agents to quickly test your hardware setup. The `zero` agent does nothing, while the `random` agent sends random actions to the robot.

```bash
srb real_agent zero --env waypoint_navigation
```

```bash
srb real_agent random --env waypoint_navigation
```

### Fine-Tuning on Real Data

> **Note:** While SRB supports fine-tuning on real data via the `srb real_agent train --continue` command, this workflow is still under active development and should be considered experimental.

## 5. Creating Custom Hardware Interfaces

You can easily support custom sensors or actuators. To create a new interface, add a new Python file in the [`sim_to_real/hardware`](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/srb/interfaces/sim_to_real/hardware) directory. Your new class should inherit from the `HardwareInterface` base class.

You will need to implement a few key methods:

- `start` to initialize your hardware connection.
- `apply_action` to send commands.
- `observation` to get sensor data.
- `close` to clean up connections.

The system will discover your new interface automatically, making it available to the `--hardware` flag.
