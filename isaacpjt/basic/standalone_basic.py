from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()

step_count = 0
was_playing = True   # 직전 프레임에 재생 중이었는지

while simulation_app.is_running():
    if world.is_playing():
        if not was_playing:
            # 정지 -> 재생으로 막 전환된 시점에만 reset
            world.reset()
            step_count = 0
            was_playing = True
            print(f'[리셋] Play 시작 -> step_count = {step_count}')

        world.step(render=True)
        time.sleep(0.01)
        step_count += 1

        if step_count == 300:
            cube_prim.set_world_pose(position=[0.0, 0.0, 1.0])
            print(f'[이동] 큐브 순간이동')

        if step_count % 100 == 0 and step_count > 1:
            print(f'step: {step_count}')

        if step_count >= 500:
            print('step_count: 500 이상 -> 시뮬레이션 종료')
            break   # while 루프 탈출 -> 시뮬레이션 완전 종료
    else:
        was_playing = False   # 정지 상태에서는 reset을 호출하지 않음
        simulation_app.update()

simulation_app.close()