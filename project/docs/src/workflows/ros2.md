# ROS 2 Workflow

The ROS 2 interface of SRB allows you to interact with the simulation environment using standard ROS 2 middleware interfaces. This enables you to develop and validate autonomous systems across a wide range of diverse scenarios through seamless integration with the rest of your ROS 2 or Space ROS nodes.

There are two options for using the ROS 2 interface of SRB:

1. **Active ROS 2 Agent**: A specific `srb agent ros` workflow that enables full control over the simulation through the ROS 2 middleware, including actions of robots (control commands)
1. **Passive ROS 2 Interface**: Any workflow can be configured to use the ROS 2 interface for publishing all available data and interacting with the simulation through services, but the actions are controlled via the selected workflow (teleoperation, random agent, RL training and evaluation, ...)

## 1. Active ROS 2 Agent

This workflow is specifically designed for full control over the simulation, including the actions of robots (control commands). It is available via the `srb agent ros` command, and works with all SRB environments. Let's try it with the `peg_in_hole_assembly_visual` task:

```bash
srb agent ros -e peg_in_hole_assembly_visual
```

### List Topics

Once the simulation is running, you can open another terminal and list all available ROS 2 topics:

```bash
ros2 topic list
```

> **Hint:** For Docker installation, you can use `.docker/join.bash` to enter the running container if you don't have a local ROS 2 setup available.

<details>
<summary>You will see the following list of topics: <i>(click to expand)</i></summary>

```
/clock
/parameter_events
/rosout
/srb/env0/action/cmd_vel
/srb/env0/action/event
/srb/env0/cam_base/camera_info
/srb/env0/cam_base/image_depth
/srb/env0/cam_base/image_rgb
/srb/env0/cam_base/pointcloud
/srb/env0/cam_scene/camera_info
/srb/env0/cam_scene/image_depth
/srb/env0/cam_scene/image_rgb
/srb/env0/cam_scene/pointcloud
/srb/env0/cam_wrist/camera_info
/srb/env0/cam_wrist/image_depth
/srb/env0/cam_wrist/image_rgb
/srb/env0/cam_wrist/pointcloud
/srb/env0/end_effector/joint_pos
/srb/env0/end_effector/joint_states
/srb/env0/reward
/srb/env0/reward/penalty_action_rate
/srb/env0/reward/penalty_undesired_robot_contacts
/srb/env0/reward/reward_align_peg_to_hole_primary
/srb/env0/reward/reward_align_peg_to_hole_secondary
/srb/env0/reward/reward_distance_end_effector_to_obj
/srb/env0/reward/reward_distance_peg_to_hole_bottom
/srb/env0/reward/reward_distance_peg_to_hole_entrance
/srb/env0/reward/reward_grasp
/srb/env0/reward/reward_lift
/srb/env0/robot/delta_twist
/srb/env0/robot/joint_states
/srb/env0/terminated
/srb/env0/truncated
/srb/envs/action/cmd_vel
/srb/envs/action/event
/srb/envs/end_effector/joint_pos
/srb/envs/robot/delta_twist
/tf
/tf_static
```

</details>

Most topics fall under two namespaces:

- `/srb/env[0-N]/...`: Environment-specific topics that are unique to each environment instance
- `/srb/envs/...`: Global topics that apply to all environment instances

Let's break down the most important topics:

| Topic                           | Message Type             | Description                               |
| ------------------------------- | ------------------------ | ----------------------------------------- |
| `/clock`                        | `rosgraph_msgs/Clock`    | Simulation time                           |
| `/tf`, `/tf_static`             | `tf2_msgs/TFMessage`     | Transformations for all scene entities    |
| **Control**                     |                          |                                           |
| `**/{robot_name}/{action_name}` | Robot-specific           | Action-specific control                   |
| `**/action/cmd_vel`             | `geometry_msgs/Twist`    | Universal command (task-specific mapping) |
| `**/action/event`               | `std_msgs/Bool`          | Universal event (task-specific mapping)   |
| **Perception**                  |                          |                                           |
| `**/[{sensor_name}]/{data}`     | Sensor-specific          | Sensor data of all scene sensors          |
| `**/{robot_name}/joint_states`  | `sensor_msgs/JointState` | Joint states of all scene articulations   |
| **Environment (MDP)**           |                          |                                           |
| `**/reward`                     | `std_msgs/Float32`       | Total reward                              |
| `**/reward/{term_name}`         | `std_msgs/Float32`       | Reward components                         |
| `**/terminated`                 | `std_msgs/Bool`          | Episode termination flag                  |
| `**/truncated`                  | `std_msgs/Bool`          | Episode truncation flag                   |

### Subscribe to Topics

You can subscribe to topics to receive updates from the simulation. For example, you can echo the reward signal of the first environment instance:

```bash
ros2 topic echo /srb/env0/reward
```

```
data: 2.36441707611084
---
data: 2.3652756214141846
---
data: 2.36613130569458
---
data: 2.3669850826263428
---
```

### Publish to Topics

You can also send control commands to the simulation by publishing to topics. For example, you can move robots in all environment instances by publishing a twist command:

```bash
ros2 topic pub --once /srb/envs/action/cmd_vel geometry_msgs/Twist "{linear: {z: -0.2}, angular: {z: 0.785}}"
```

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/NU8t8WfrMSc?si=eTHoGjNMIhWPlLIW&mute=1&autoplay=1&loop=1&playlist=NU8t8WfrMSc" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## 2. Passive ROS 2 Interface

Alternatively, you can use the ROS 2 interface with any agent type by specifying the `--interface ros` flag. This allows you to collect sensor data and interact with the simulation instances while controlling the agent through the selected workflow. Let's try it with the `rand` agent in the `sample_collection_multi_visual` environment with 8 parallel instances:

```bash
srb agent rand -e sample_collection_multi_visual env.num_envs=8 --interface ros
```

> **Note:** No Hydra overrides can be specified directly after the `--interface X` option because it accepts one or more interfaces.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/_wPt32cVOY4?si=Rvlalg0z1SLFFDrQ&mute=1&autoplay=1&loop=1&playlist=_wPt32cVOY4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Call Services

Upon listing the available services, you will see that each environment instance has a `/srb/env{i}/reset` service, and a global `/srb/envs/reset` service.

```bash
ros2 service list
```

You can call these services to reset a specific environment instance or all instances at once:

```bash
ros2 service call /srb/env0/reset std_srvs/srv/Empty
```

```bash
ros2 service call /srb/envs/reset std_srvs/srv/Empty
```
