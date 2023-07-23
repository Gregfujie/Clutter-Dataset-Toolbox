import os
import numpy as np

from pcg_gazebo.generators import WorldGenerator

building_name = 'room1_room'
ceiling_name = building_name + '_ceiling'
output_dir = "/home/greg/Documents/gazeboprojects/building_gen/worlds"
world_name = "room1.world"

world_gen = WorldGenerator()

world_gen.add_engine(
    engine_name='fixed_pose',
    tag='engine1',
    models=[building_name],
    poses=[
        [0, 0, 1.25, 0, 0, 0],
    ]
)

world_gen.add_engine(
    engine_name='fixed_pose',
    tag='engine2',
    models=[ceiling_name],
    poses=[
        [0, 0, 2.6, 0, 0, 0],
    ]
)

world_gen.run_engines()
world_gen.export_world(output_dir=output_dir,
                            filename=world_name,
                            with_default_ground_plane=True,
                            with_default_sun=True)
print("end")