# `srb agent rand` — Random Agent

The `srb agent rand` command runs an environment with randomly sampled actions. This is useful for testing the environment's response to diverse inputs, identifying edge cases, and ensuring robustness in the environment implementation.

## Usage

```bash
srb agent rand --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

No additional options are available for this subcommand.

## Example

Run random agent in the `_aerial` template environment:

- Hide the simulation UI
- Use the `body_acceleration` action group

```bash
srb agent rand -e _aerial --hide_ui env.robot.actions=body_acceleration
```
