# `srb agent train` — Train Agent

The `srb agent train` command allows you to train Reinforcement Learning (RL) agents on Space Robotics Bench environments.

## Usage

```bash
srb agent train --algo ALGO --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

| Option                | Description                                | Default      |
| --------------------- | ------------------------------------------ | ------------ |
| `--algo ALGO`         | RL algorithm to use                        | **REQUIRED** |
| `--continue_training` | Continue training from last checkpoint     | `False`      |
| `--model MODEL`       | Continue training from specific checkpoint |              |

## Examples

See the [Reinforcement Learning Workflow](../workflows/reinforcement_learning.md) guide for detailed examples.
