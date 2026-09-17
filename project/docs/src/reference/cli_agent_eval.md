# `srb agent eval` — Evaluate Agent

The `srb agent eval` command allows you to evaluate trained Reinforcement Learning (RL) agents on Space Robotics Bench environments.

## Usage

```bash
srb agent eval --algo ALGO --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

| Option          | Description                               | Default      |
| --------------- | ----------------------------------------- | ------------ |
| `--algo ALGO`   | RL algorithm used for training the policy | **REQUIRED** |
| `--model MODEL` | Path to a specific checkpoint             |              |

## Examples

See the [Reinforcement Learning Workflow](../workflows/reinforcement_learning.md) guide for detailed examples.
