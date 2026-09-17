# Environment Configuration — Parallelism

One of Space Robotics Bench's most powerful features is parallel simulation, allowing you to run multiple simulation instances simultaneously. This capability is critical for:

- **Reinforcement Learning**: Collect experience at scale for faster training
- **Parameter Tuning**: Test multiple configurations simultaneously
- **Monte Carlo Sampling**: Evaluate robustness across varied scenarios
- **Batch Processing**: Process multiple scenarios in a single run

## Number of Environments

The `env.scene.num_envs` parameter controls how many parallel simulation instances are created. This parameter is aliased as `env.num_envs` for brevity:

```bash
srb agent rand -e _manipulation env.scene.num_envs=16
srb agent zero -e _manipulation env.num_envs=128
```

Each environment is a fully independent physics simulation instance, with all cross-environment interactions disabled (filtered collisions). However, **there is only one instance of the rendering engine**, which means that a visual sensor in one environment will see entities from all environments. A simple workaround for this limitation is to increase the spacing between environments and clip the maximum sensor range.

## Environment Spacing

The `env.scene.env_spacing` parameter controls the spatial distance between environments. This parameter is aliased as `env.spacing` for brevity:

```bash
srb agent rand -e _manipulation env.num_envs=64 env.scene.spacing=20.0
srb agent rand -e _manipulation env.num_envs=32 env.spacing=10.0
```

## Environment Stacking

The `env.stack` parameter controls whether environments share assets or have independent assets:

### Independent Environments (`env.stack=false`) \[Default\]

- ⚠️ Each environment has a unique scene and position
- ✅ Greater visual and physical diversity
- ✅ Supports tasks with visual sensors
- ❗ Slower to initialize
- ❗ Higher memory usage

```bash
srb agent rand -e _manipulation env.num_envs=16 env.stack=false
```

### Stacked Environments (`env.stack=true`)

- ⚠️ All environments share the same scenery and position
- ✅ Faster to initialize
- ✅ Lower memory usage
- ❗ Less environmental diversity
- ❗ Does not support tasks with visual sensors

```bash
srb agent rand -e _manipulation env.num_envs=16 env.stack=true
```

> **Note**: Non-visual `locomotion_velocity_tracking` task defaults to `env.stack=true`.
