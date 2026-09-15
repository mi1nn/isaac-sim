from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

INITIAL_POSITION = np.array([0.0, 0.0, 0.15])
TELEPORT_POSITION = np.array([0.0, 0.0, 1.0])

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=INITIAL_POSITION,
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()

step_count = 0
teleported = False

previous_playing = False
previous_stopped = world.is_stopped()

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)
    step_count += 1
    
    is_playing = world.is_playing()
    is_stopped = world.is_stopped()
    
    if is_stopped and not previous_stopped:   

        print(f"[리셋] Play 시작 -> Step = {step_count}")
        
        step_count = 0
        teleported = False

        # 큐브 초기 위치 복원
        cube_prim.set_world_pose(
            position=INITIAL_POSITION
        )
        
        cube_prim.set_linear_velocity(
            np.array([0.0, 0.0, 0.0])
        )
        
        cube_prim.set_angular_velocity(
            np.array([0.0, 0.0, 0.0])
        )
    
    if is_playing and not previous_playing:

        print(f"[리셋] Play 시작 -> Step = {step_count}")

    if is_playing:

        step_count += 1

        if step_count % 100 == 0:
            print(f"Step : {step_count}")

        if step_count == 300 and not teleported:

            print("[TELEPORT] Cube 이동")

            cube_prim.set_world_pose(
                position=TELEPORT_POSITION
            )

            # 순간이동할 때 기존 속도 제거
            cube_prim.set_linear_velocity(
                np.array([0.0, 0.0, 0.0])
            )

            cube_prim.set_angular_velocity(
                np.array([0.0, 0.0, 0.0])
            )

            teleported = True

    previous_playing = is_playing
    previous_stopped = is_stopped
    
simulation_app.close()