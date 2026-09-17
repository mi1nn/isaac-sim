# `srb ls` — List Assets and Environments

The `srb ls` command allows you to list all registered assets, action groups, and environments in Space Robotics Bench.

## Usage

```bash
srb ls [options]
```

### Options

| Argument             | Description                            | Default |
| -------------------- | -------------------------------------- | :-----: |
| \*categories         | Sequence of categories to list         | `[all]` |
| `-a`/`--show_hidden` | Show hidden entities (`*_visual` envs) | `False` |

## Output

The command produces a tabular output with the requested registered entities.

> **Hint:** In VSCode, the table is interactive and provides clickable links to the source code.

### Example Output (Truncated)

```
                                                          Assets of the Space Robotics Bench
┏━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ # ┃ Name                        ┃ Type    ┃ Subtype            ┃ Parent Class          ┃ Asset Config    ┃ Path                             ┃
┡━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 1 │ mars_surface                │ scenery │ terrain            │ Terrain               │ AssetBaseCfg    │ planetary_surface.py             │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┤
│ 2 │ cargo_bay                   │ object  │ payload            │ Payload               │ AssetBaseCfg    │ payload/cargo_bay.py             │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┤
│ 3 │ scoop                       │ object  │ tool               │ Tool                  │ RigidObjectCfg  │ tool/scoop.py                    │
│ 4 │ shadow_hand                 │ object  │ tool               │ ActiveTool            │ ArticulationCfg │ tool/shadow_hand.py              │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┤
│ 5 │ sample_tube                 │ object  │ common             │ Object                │ RigidObjectCfg  │ sample.py                        │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┤
│ 6 │ franka                      │ robot   │ manipulator        │ SerialManipulator     │ ArticulationCfg │ manipulation/franka.py           │
│ 7 │ random_ur_manipulator       │ robot   │ manipulator        │ SerialManipulator     │ ArticulationCfg │ manipulation/universal_robots.py │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┤
│ 8 │ random_unitree_quadruped    │ robot   │ mobile_robot       │ LeggedRobot           │ ArticulationCfg │ mobile/unitree.py                │
├───┼─────────────────────────────┼─────────┼────────────────────┼───────────────────────┼─────────────────┼──────────────────────────────────┤
│ 9 │ unitree_g1                  │ robot   │ mobile_manipulator │ Humanoid              │ ArticulationCfg │ mobile_manipulation/unitree.py   │
│ . │ ...                         │ ...     │ ...                │ ...                   │ ...             │ ...                              │
└───┴─────────────────────────────┴─────────┴────────────────────┴───────────────────────┴─────────────────┴──────────────────────────────────┘

             Action Groups of the Space Robotics Bench
┏━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ # ┃ Name                          ┃ Path                       ┃
┡━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 1 │ body_acceleration             │ common/body.py             │
│ 2 │ body_acceleration_relative    │ common/body.py             │
│ 3 │ joint_position                │ common/joint.py            │
│ 4 │ joint_position_relative       │ common/joint.py            │
│ 5 │ joint_position_binary         │ common/joint.py            │
│ 6 │ joint_velocity                │ common/joint.py            │
│ 7 │ joint_velocity_binary         │ common/joint.py            │
│ 8 │ joint_effort                  │ common/joint.py            │
│ 9 │ inverse_kinematics            │ manipulation/task_space.py │
│ . │ ...                           │ ...                        │
└───┴───────────────────────────────┴────────────────────────────┘

                                                        Environments of the Space Robotics Bench
┏━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ # ┃ ID                               ┃ Entrypoint                   ┃ Config                             ┃ Path                                      ┃
┡━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 1 │ _manipulation <template>         │ Task(ManipulationEnv)        │ TaskCfg(ManipulationEnvCfg)        │ manipulation/_manipulation                │
│ 2 │ sample_collection                │ Task(ManipulationEnv)        │ TaskCfg(ManipulationEnvCfg)        │ manipulation/sample_collection            │
│ 3 │ _aerial <template>               │ Task(AerialEnv)              │ TaskCfg(AerialEnvCfg)              │ mobile/_aerial                            │
│ 4 │ _ground <template>               │ Task(GroundEnv)              │ TaskCfg(GroundEnvCfg)              │ mobile/_ground                            │
│ 5 │ _orbital <template>              │ Task(OrbitalEnv)             │ TaskCfg(OrbitalEnvCfg)             │ mobile/_orbital                           │
│ 6 │ locomotion_velocity_tracking     │ Task(GroundEnv)              │ TaskCfg(GroundEnvCfg)              │ mobile/locomotion_velocity_tracking       │
│ 7 │ _aerial_manipulation <template>  │ Task(AerialManipulationEnv)  │ TaskCfg(AerialManipulationEnvCfg)  │ mobile_manipulation/_aerial_manipulation  │
│ 8 │ _ground_manipulation <template>  │ Task(GroundManipulationEnv)  │ TaskCfg(GroundManipulationEnvCfg)  │ mobile_manipulation/_ground_manipulation  │
│ 9 │ _orbital_manipulation <template> │ Task(OrbitalManipulationEnv) │ TaskCfg(OrbitalManipulationEnvCfg) │ mobile_manipulation/_orbital_manipulation │
│ . │ ...                              │ ...                          │ ...                                │ ...                                       │
└───┴──────────────────────────────────┴──────────────────────────────┴────────────────────────────────────┴───────────────────────────────────────────┘
```
