# `srb agent zero` — Zero Agent

The `srb agent zero` command runs an environment with zero-valued actions. For action spaces with equilibrium at zero, the environment will evolve solely based on its internal dynamics. This is useful for testing the environment and observing the behavior of the agent without any control input.

## Usage

```bash
srb agent zero --env ENV_ID [options]
```

## Options | [Shared Agent Options](cli.md#shared-agent-options)

No additional options are available for this subcommand.

## Example

Run zero agent in the `_ground` template environment:

- Create 4 parallel environment instances
- Use the Cassie bipedal robot

```bash
srb agent zero -e _ground env.num_envs=4 env.robot=cassie
```
