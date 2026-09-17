# Agent Configuration

The Space Robotics Bench leverages [Hydra](https://hydra.cc) for managing agent configurations across different robot learning frameworks. This document provides an overview of the configuration structure and common parameters for training agents in SRB.

## Default Hyperparameters

The default hyperparameters for all algorithms and environments are available under the [space_robotics_bench/hyperparams](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/hyperparams) directory. If you do not wish to use Hydra, you can directly modify these files before training your agent.

> **Note:** The available hyperparameters and their structure are specific to each framework and algorithm.

## Modifying Configurations

You can modify agent configurations in several ways:

### 1. Command-Line Overrides

```bash
srb agent <WORKFLOW> --algo <ALGO> --env <ENV> \
    agent.learning_rate=0.0001 \
    agent.batch_size=64 \
    ...
```

### 2. Configuration Files

<!-- TODO[docs]: Add instructions for agent configuration files -->

<div class="warning">

#### This section is under construction. Please check back later for updates.

</div>
