# `srb agent teleop` — Teleoperate Agent

The `srb agent teleop` command allows you to manually control robots in the environment using various input devices. This is useful for testing the environment and observing the behavior of the agent under specific control inputs.

## Usage

```bash
srb agent teleop --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

| Option                        | Description                               | Default      |
| ----------------------------- | ----------------------------------------- | ------------ |
| `--teleop_device` \[DEV ...\] | Interface (`keyboard`, `spacemouse`, ...) | `[keyboard]` |
| `--pos_sensitivity` VAL       | Translation sensitivity                   | `10.0`       |
| `--rot_sensitivity` VAL       | Rotation sensitivity                      | `40.0`       |
| `--invert_controls`           | Invert control schema                     | `False`      |
| **Teleoperation via Policy**  |                                           |              |
| `--algo ALGO`                 | RL algorithm of the policy                |              |
| `--model MODEL`               | Path to the policy checkpoint             |              |

## Teleoperation Modes

The `teleop` subcommand supports 2 modes of operation based on the environment action space:

- [**Direct Teleoperation**](#direct-teleoperation)
- [**Teleoperation via Policy**](#teleoperation-via-policy)

### Direct Teleoperation

Direct teleoperation is available for environments with action spaces that can be mapped into high-level control commands, such as the desired end-effector pose or acceleration of a mobile robot.

#### Examples

Teleoperate a robot in the `_manipulation` template environment:

- Use the `ur10` manipulator

```bash
srb agent teleop -e _manipulation env.robot=ur10
```

Teleoperate a robot in the `_orbital` template environment:

- Use the `spacemouse` input teleoperation device

```bash
srb agent teleop -e _orbital --teleop_device spacemouse
```

### Teleoperation via Policy

Some action spaces do not support direct teleoperation due to their complexity or dimensionality, such as joint-space control of a humanoid robot. In such cases, you can still teleoperate the agent by providing a trained policy (only RL agents are currently supported). In this case, command-like observations are driven by the teleoperation device, and the policy generates the corresponding actions.

#### Examples

> Reference: [Reinforcement Learning Workflow](../workflows/reinforcement_learning.md)\
> Reference: [`srb agent train` — Train Agent](../reference/cli_agent_train.md)

First, you need to train an RL agent using the `srb agent train` command. Let's demonstrate this with the `locomotion_velocity_tracking` environment using the `dreamer` algorithm:

```bash
srb agent train --headless --algo dreamer -e locomotion_velocity_tracking env.num_envs=256
```

After you have successfully trained a policy, you can teleoperate the agent:

```bash
srb agent teleop --algo dreamer -e locomotion_velocity_tracking
```

By default, the policy will be loaded from the latest checkpoint. However, you can specify a specific checkpoint using the `--model` option.

```bash
srb agent teleop --algo dreamer -e locomotion_velocity_tracking --model space_robotics_bench/logs/locomotion_velocity_tracking/dreamer/...
```
