import numpy as np
import math
from pcg_gazebo.generators import WorldGenerator
import random


class WallGrids:

    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.wall_grids = []
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = max(0.1, z_min)
        self.z_max = z_max
        self.cell_size = 1

    def add_wall(self, position, scale, yaw):
        if abs(yaw) > 0.1 and abs(abs(yaw) - 3.14) > 0.1 and abs(abs(yaw) - 1.57) > 0.1:
            return
        if scale[2] < 1.9:
            return
        if scale[0] < 2 and scale[1] < 2:
            return
        else:
            temp_grids = []
            if scale[0] > scale[1]:
                x_small = round(-scale[0] / 2 + 0.15, 1)
                x_big = round(scale[0] / 2 - 0.15, 1)
                length = math.floor((x_big - x_small) / self.cell_size)
                for i in range(length):
                    x = x_small + i * self.cell_size + self.cell_size / 2
                    y = scale[1] / 2 + 0.1
                    temp_grids.append([x, y, 1])
                    temp_grids.append([x, -y, -1])

            else:
                y_small = round(-scale[1] / 2 + 0.15, 1)
                y_big = round(scale[1] / 2 - 0.15, 1)
                length = math.floor((y_big - y_small) / self.cell_size)
                for i in range(length):
                    y = y_small + i * self.cell_size + self.cell_size / 2
                    x = scale[0] / 2 + 0.1
                    temp_grids.append([x, y, 1])
                    temp_grids.append([-x, y, -1])

            trans = np.array([[math.cos(yaw), math.sin(yaw)],
                              [-math.sin(yaw), math.cos(yaw)]])
            for each in temp_grids:
                temp_p = np.array([[each[0]], [each[1]]])
                result_p = trans @ temp_p
                trans_p = [result_p[0][0] + position[0], result_p[1][0] + position[1]]
                if trans_p[0] > self.x_max or trans_p[0] < self.x_min or trans_p[1] > self.y_max or trans_p[
                    1] < self.y_min:
                    continue
                height = math.floor((self.z_max - self.z_min) / self.cell_size)
                for i in range(height):
                    roll = random.randint(-10, 10) / 50
                    pitch = random.randint(-5, 5) / 50
                    slate_yaw = pi_2_pi(yaw + 1.5757)
                    self.wall_grids.append(
                        [trans_p[0], trans_p[1], self.z_min + self.cell_size / 2 + i * self.cell_size, roll, pitch,
                         slate_yaw])

    def get_grids(self):
        return self.wall_grids


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


world_path = "/home/greg/Documents/gazeboprojects/pcg_gazebo_test/test2.world"
output_path = "/home/greg/Documents/gazeboprojects/pcg_gazebo_test/gened_world"
output_file = "final_test.world"
room_model_name = "demo_clean"
ceiling_model_name = "new_ceiling_clone"
world_gen = WorldGenerator()
world_gen.init_from_sdf(world_path)
pcg_room = world_gen.world.models[room_model_name]
ceiling = world_gen.world.models[ceiling_model_name]
x_min, x_max, y_min, y_max, z_min, z_max = 100, -100, 100, -100, 100, -100

for each_link in pcg_room.link_names:
    temp_link = pcg_room.links[each_link]
    link_size = temp_link.visuals[0].geometry._sdf.size.value
    x, y, z = temp_link.pose.position

    x_min = min(x_min, x)
    x_max = max(x_max, x)
    y_min = min(y_min, y)
    y_max = max(y_max, y)
    z_min = min(z_min, z)
    z_max = max(z_max, z)

x_max = round(x_max, 1)
x_min = round(x_min, 1)
y_max = round(y_max, 1)
y_min = round(y_min, 1)
z_min = world_gen.world.models["ground_plane"].pose.position[2]
z_max = max(z_max, ceiling.pose.position[2])
z_max = round(z_max, 1)
z_min = round(z_min, 1)

test = WallGrids(x_min, x_max, y_min, y_max, z_min, z_max)

for each_link in pcg_room.link_names:
    temp_link = pcg_room.links[each_link]
    link_size = temp_link.visuals[0].geometry._sdf.size.value
    test.add_wall(temp_link.pose.position, link_size, temp_link.pose.rpy[2])

grids = test.get_grids()
index = [i for i in range(len(grids))]
random.shuffle(index)
grids_to_gen = [grids[index[i]] for i in range(int(len(grids) * 3 / 5))]

world_gen.add_engine(
    engine_name='fixed_pose',
    tag='engine1',
    models=['wall_slate'],
    poses=grids_to_gen
)

world_gen.run_engines(attach_models=True)
world_gen.export_world(output_dir=output_path,
                            filename=output_file,
                            with_default_ground_plane=False,
                            with_default_sun=False)
print("end")