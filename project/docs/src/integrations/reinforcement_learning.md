# Integration — Reinforcement Learning

The Space Robotics Bench is designed with Robot Learning research in mind, with a particular emphasis on Reinforcement Learning (RL). All environments follow the standard Gymnasium API, making them compatible with most modern RL frameworks. This architecture enables rapid prototyping, training, and evaluation of policies across diverse space robotics tasks, which makes SRB particularly suitable for comparing the generalization capabilities of novel RL algorithms.

<iframe style="width:100%;aspect-ratio:5/4" src="https://www.youtube.com/embed/ulxwdki9vzs?si=_ftFARI9fDwxio1Q&mute=1&autoplay=1&loop=1&playlist=ulxwdki9vzs" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Motivation

RL has emerged as a promising approach for developing autonomous behaviors in complex robotic systems, especially for space applications where manual control may be limited by communication delays or environmental uncertainties. SRB provides a comprehensive set of environments that are specifically designed to facilitate RL research in space robotics, offering several key features:

- **Diverse Tasks** - Collection of various space robotics tasks that range from simple navigation to complex mobile manipulation
- **Parallel Training** - All environments support parallel training across multiple instances for efficient data collection
- **Procedural Generation & Domain Randomization** - Generalization capabilities of RL algorithms can be put to the test using procedurally generated environments that are randomized across multiple dimensions
- **Unified Benchmarking** - RL algorithms can be compared across a set of reproducible tasks with consistent evaluation metrics

## Available Integrations

We provide official integrations with the following RL frameworks:

- **[Dreamer](https://danijar.com/project/dreamerv3) \[JAX\]** - Model-based RL algorithm
- **[Stable-Baselines3 (SB3) & SBX](https://stable-baselines3.readthedocs.io) \[PyTorch & JAX\]** - Popular implementation of RL algorithms
- **[skrl](https://skrl.readthedocs.io) \[PyTorch & JAX\]** - Implementation of single- and multi-agent RL algorithms

## How to Get Started?

If you want to start using the Space Robotics Bench for Reinforcement Learning, follow these steps:

1. [Installation](../getting_started/install.md)
1. [Basic Usage](../getting_started/basic_usage.md)
1. [RL Workflow](../workflows/reinforcement_learning.md)
