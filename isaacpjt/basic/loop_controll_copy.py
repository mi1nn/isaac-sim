from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

INITIAL_POSITION = np.array([0.0, 0.0, 0.15])
TELEPORT_POSITION = np.array([0.0, 0.0, 1.0])

cube_prim = DynamicCuboid(
    prim_path="/World/RedCube",
    name="red_cube",
    position=INITIAL_POSITION,
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)

world.scene.add_default_ground_plane()
world.scene.add(cube_prim)

world.reset()

step_count = 0
teleported = False

was_stopped = world.is_stopped()

while simulation_app.is_running():

    world.step(render=True)

    is_playing = world.is_playing()
    is_stopped = world.is_stopped()

    if was_stopped and is_playing:

        print(f"[리셋] Play 시작 -> step_count = {step_count}")

        step_count = 0
        teleported = False

        cube_prim.set_world_pose(
            position=INITIAL_POSITION
        )

    if is_playing:

        step_count += 1

        if step_count % 100 == 0:
            print(f"Step : {step_count}")

        if step_count >= 300 and not teleported:

            cube_prim.set_world_pose(
                position=TELEPORT_POSITION
            )

            teleported = True

            print("[이동] 큐브 순간이동")

    was_stopped = is_stopped

simulation_app.close()