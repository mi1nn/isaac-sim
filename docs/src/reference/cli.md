# Command Line Interface (CLI)

The Space Robotics Bench provides a comprehensive command-line interface that allows you to interact with most aspects of the framework through a unified interface. The CLI is structured as a set of subcommands, each with its own set of options.

```bash
srb <subcommand> [options]
```

If you need help with a specific subcommand, you can use the `-h`/`--help` flag:

```bash
srb --help
srb <subcommand> -h
```

You can also `TAB` your way through the available subcommands and options via autocompletion:

```bash
srb <TAB> <TAB>
```

## Subcommands

The following subcommands are available in the CLI:

| Subcommand                                                        | Description                                            |
| ----------------------------------------------------------------- | ------------------------------------------------------ |
| [`agent`](#agent-subcommands)                                     | Agent subcommands ([listed below](#agent-subcommands)) |
| [`ls`](cli_ls.md)                                                 | List registered assets, action groups and environments |
| [`repl`](cli_repl.md)                                             | Enter Python REPL with Isaac Sim initialized           |
| [`test`](../development/testing.md#srb-test--run-tests)           | Run automated tests                                    |
| [`docs`](../development/documentation.md#srb-docs--local-preview) | Build documentation                                    |
| [`gui`](gui.md#srb-gui--launch-gui)                               | Launch the Graphical User Interface                    |

## Agent Subcommands

The `agent` subcommand is further separated into agent-specific subcommands:

```bash
srb agent <agent_subcommand> [options]
```

| Agent Subcommand                | Description                              |
| ------------------------------- | ---------------------------------------- |
| [`zero`](cli_agent_zero.md)     | Run agent with zero-valued actions       |
| [`rand`](cli_agent_rand.md)     | Run agent with random actions            |
| [`teleop`](cli_agent_teleop.md) | Manually teleoperate the agent           |
| [`ros`](cli_agent_ros.md)       | Interface with ROS 2 or Space ROS        |
| [`train`](cli_agent_train.md)   | Train Reinforcement Learning (RL) agents |
| [`eval`](cli_agent_eval.md)     | Evaluate trained RL agents               |

### Shared Agent Options

The following options are shared across all `agent` subcommands:

| Argument                    | Description                    |    Default     |
| --------------------------- | ------------------------------ | :------------: |
| `-e`/`--env`/`--task` ENV   | ID of the environment          |  **REQUIRED**  |
| `--headless`                | Run simulation without display |    `False`     |
| `--hide_ui`                 | Disable simulation UI          |    `False`     |
| `--interface` \[IFACE ...\] | Interfaces to enable           |      `[]`      |
| `--logdir`/`--logs` PATH    | Path to logging directory      | `SRB_LOGS_DIR` |
| `--video`                   | Enable video recording         |    `False`     |
| `--video_length`            | Duration of recorded videos    |  `1000` steps  |
| `--video_interval`          | Interval of video recordings   | `10000` steps  |
