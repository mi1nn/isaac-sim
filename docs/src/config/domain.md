# Environment Configuration — Domain

The `env.domain` parameter configures the planetary or orbital environment for your simulation. This affects gravity, lighting conditions, terrain type, atmospheric properties, and visual appearance.

## Available Domains

Space Robotics Bench currently supports the following domains:

| Domain             | Description                             |
| ------------------ | --------------------------------------- |
| `moon` \[default\] | Lunar surface                           |
| `mars`             | Martian surface                         |
| `earth`            | Earth surface                           |
| `asteroid`         | Low-gravity body with irregular terrain |
| `orbit`            | Orbital microgravity environment        |

## Usage

You can specify the domain via the command line:

```bash
# Run a simulation on Mars
srb agent teleop --env sample_collection env.domain=mars
```

## Domain-Specific Effects

Each domain affects various aspects of the simulation:

### Physical Properties

- **Gravity magnitude**: Affects object dynamics, robot mobility and manipulation requirements

### Visual Properties

- **Lighting**: Light intensity, color temperature, and angular diameter
- **Skydome**: Background appearance and intensity
- **Atmosphere**: Fog intensity and color gradient (if applicable)

### Asset Selection

Some assets are domain-specific and will only appear for compatible domains. When an asset is registered, it can specify the domains it supports through the `DOMAINS` class attribute.
