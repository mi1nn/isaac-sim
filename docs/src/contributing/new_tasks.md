# Contributing — New Tasks

The process of introducing a new environment into the Space Robotics Bench is intended to be straightforward, with a limited amount of boilerplate that you need to write yourself. This guide will walk you through the primary steps for creating a new SRB task.

## 1. Duplicate a Template

Navigate to the [srb/tasks](https://github.com/AndrejOrsula/space_robotics_bench/tree/main/srb/tasks) directory in your local repository. Then, duplicate one of the existing task templates and rename it to your desired task name. It is recommended that you keep your task in the same root directory as the template in order to simplify the registration process. You should select a template based on the type of task and scenario you wish to create:

| Template                                                                                                                                              | Description                                  |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------- |
| [`_manipulation`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/manipulation/_manipulation/task.py)                        | Fixed-base manipulation with robotic arms    |
| [`_ground`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile/_ground/task.py)                                          | Ground traversal on planetary surfaces       |
| [`_aerial`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile/_aerial/task.py)                                          | Aerial navigation above planetary surfaces   |
| [`_orbital`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile/_orbital/task.py)                                        | Spaceflight maneuvers                        |
| [`_ground_manipulation`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile_manipulation/_ground_manipulation/task.py)   | Mobile manipulation with ground-based robots |
| [`_aerial_manipulation`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile_manipulation/_aerial_manipulation/task.py)   | Mobile manipulation with flying robots       |
| [`_orbital_manipulation`](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/srb/tasks/mobile_manipulation/_orbital_manipulation/task.py) | Mobile manipulation with spacecraft          |

## 2. Modify the Environment

Now, it is time to be creative and modify the environment to suit your needs. You can adjust the following components:

- **Scene**: Change the assets and layout of the environment to match your scenario.
- **Robot**: Specify a category of robots that are suitable for performing the task.
- **Task**: Define the task-specific logic and objectives.
  - **Observation Space**: Define the observation space for the task.
  - **Reward Function**: Define the reward function for the task.
  - **Termination Condition**: Define the termination condition for the task.

As this step is very task-specific, don't hesitate to reach out to the community for help or guidance.

## 3. Debug the Task

> Reference: [`srb agent teleop` — Teleoperate Agent](../reference/cli_agent_teleop.md)\
> Reference: [`srb agent zero` — Zero Agent](../reference/cli_agent_zero.md)\
> Reference: [`srb agent rand` — Random Agent](../reference/cli_agent_rand.md)

While developing your task, it is essential to test it to ensure that it behaves as expected. Depending on the action space of your robot, you can either use `teleop` or `zero`/`rand` commands to control the robot. It is also recommended to enable debug visualizations to help you better understand the behavior of the task, and ensure that the environment works with parallel instances:

```bash
srb agent teleop -e <your_task_name> env.debug_vis=true env.num_envs=4
```

```bash
srb agent zero -e <your_task_name> env.debug_vis=true env.num_envs=4
```

```bash
srb agent rand -e <your_task_name> env.debug_vis=true env.num_envs=4
```

## 4. Use the Task in a Workflow

> Reference: [Workflows](../workflows/index.md)

Now, you are ready to use your task in your desired workflow. Whether you are experimenting with training RL agents or developing a future space mission, your task can be integrated into any workflow that the Space Robotics Bench provides.

Feel free to show & tell us about your task in the community. We are excited to see what you have created!
