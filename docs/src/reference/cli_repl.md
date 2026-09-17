# `srb repl` — Enter Python REPL

The `srb repl` command launches an interactive Python REPL (Read-Eval-Print Loop) with Isaac Sim initialized and `srb` module preloaded. This is useful for exploration, debugging, and ad-hoc experimentation with the simulation environment.

## Usage

```bash
srb repl [options]
```

### Options

| Argument     | Description                    | Default |
| ------------ | ------------------------------ | :-----: |
| `--headless` | Run simulation without display | `False` |
| `--hide_ui`  | Disable simulation UI          | `False` |

## Example

Enter the Python REPL with Isaac Sim initialized:

```bash
srb repl
```

Load the `sample_collection` environment with 4 parallel instances:

```python
env_cfg = srb.tasks.manipulation.sample_collection.TaskCfg(num_envs=4)
env = srb.tasks.manipulation.sample_collection.Task(env_cfg)
```

Import PyTorch:

```python
import torch
```

Step the environment 50 times:

```python
env.reset()
for _ in range(50):
    env.step(action=torch.tensor(env.action_space.sample(), device=env.device))
```
