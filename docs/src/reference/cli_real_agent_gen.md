# `srb real_agent gen` — Sim-to-Real Bridge

The `srb real_agent gen` command inspects a simulated Gymnasium environment from the Space Robotics Bench and automatically generates its lightweight, real-world counterpart. This new environment is registered in the `srb_real/` namespace and does not depend on Isaac Sim, allowing it to run directly on hardware.

## Usage

```bash
srb real_agent gen --env ENV_ID [options]
```

## Options

| Option                     | Description                                                                                                                              | Default      |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- | ------------ |
| `-e/--env/--task ENV_ID`   | Name of the simulated environment to generate the real-world counterpart for. Can be `ALL` to generate for every registered environment. | **REQUIRED** |
| `--hardware/--hw [HW ...]` | Sequence of default hardware interfaces to link with the generated environment.                                                          | `[]`         |

## Examples

The following command generates a real-world environment for the `waypoint_navigation` task, configured for the `leo_rover` and linked to specific hardware interfaces for control, sensing, and middleware services.

```bash
srb real_agent gen --env waypoint_navigation env.robot=leo_rover --hardware ros_cmd_vel ros_tf ros_mw
```

This will create a new Python file and register the `srb_real/waypoint_navigation` environment, making it available to other `srb real_agent` subcommands.

See the [Sim-to-Real Transfer](../workflows/sim_to_real.md) guide for more detailed examples and context.
