# Environment Configuration — Robot

The `env.robot` parameter allows you to specify which robot to use in your simulation. This is one of the most powerful configuration options, enabling you to change not just the robot model but also its components, payloads, and end effectors.

## Robot Selection

The simplest way to select a robot is by specifying its name:

```bash
srb agent rand -e _manipulation env.robot=ur10
```

```bash
srb agent rand -e _ground env.robot=unitree_go1
```

```bash
srb agent rand -e _aerial env.robot=crazyflie
```

### End Effector for Manipulators

You can attach specific end effectors to manipulators using the `+` syntax:

```bash
srb agent rand -e _manipulation env.robot=ur10+shadow_hand
```

To keep the default end effector while changing the base manipulator:

```bash
srb agent rand -e _manipulation env.robot=unitree_z1+
```

To only change the end effector:

```bash
srb agent rand -e _manipulation env.robot=+shadow_hand
```

### Payload for Mobile Robots

Similarly, you can attach payloads to mobile robots:

```bash
srb agent rand -e _ground env.robot=spot+cargo_bay
```

To keep the default payload while changing the base mobile robot:

```bash
srb agent rand -e _ground env.robot=anymal_d+
```

To only change the payload:

```bash
srb agent rand -e _ground env.robot=+cargo_bay
```

## Combine Mobile Manipulators

SRB supports combining any mobile robot with any manipulator for mobile manipulation environments via the `env.robot.mobile_base` and `env.robot.manipulator` parameters:

```bash
srb agent rand -e _ground_manipulation env.robot.mobile_base=unitree_a1 env.robot.manipulator=unitree_z1
```

Similarly, you can attach payloads to the mobile base and end effectors to the manipulator:

```bash
srb agent rand -e _ground_manipulation env.robot.mobile_base=anymal_d+cargo_bay env.robot.manipulator=ur10+shadow_hand
```

## Change Action Groups

Each robot and active tool (end-effector) has its associated action group that is automatically manifested in the action space of the whole system. You can change the action group of the robot or active tool using the `env.robot.actions` parameter:

```bash
srb agent rand -e _manipulation env.robot.actions=joint_position_relative
```

> **Note:** It is currently not possible to override both the robot and its action group via their registered names at the same time.
