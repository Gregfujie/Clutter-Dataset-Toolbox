import warnings
import pycubicspline

warnings.filterwarnings("ignore")

from pcg_gazebo.generators import WorldGenerator

import occupied_grids as og

from workspace import WorkSpace

# initialize and loading
world_path = "/home/greg/Documents/gazeboprojects/pcg_gazebo_test/gened_world/wall_test.world"
output_path = "/home/greg/Documents/gazeboprojects/pcg_gazebo_test/gened_world"
output_file = "final_test.world"
room_model_name = "demo_clean"
ceiling_model_name = "new_ceiling_clone"
test_world = WorldGenerator()
test_world.init_from_sdf(world_path)

print("start workspace")
# generate workspace
for each in test_world.world.models.keys():
    print(each)
pcg_room = test_world.world.models[room_model_name]
ceiling = test_world.world.models[ceiling_model_name]
ws = WorkSpace()
for each_link in pcg_room.link_names:
    temp_link = pcg_room.links[each_link]
    link_size = temp_link.visuals[0].geometry._sdf.size.value
    ws.add_wall(temp_link.pose.position, link_size, temp_link.pose.rpy[2])
ws_range = ws.get_range()
x_max = round(ws_range[1], 1)
x_min = round(ws_range[0], 1)
y_max = round(ws_range[3], 1)
y_min = round(ws_range[2], 1)
z_min = test_world.world.models["ground_plane"].pose.position[2]
z_max = max(ws_range[5], ceiling.pose.position[2])
z_max = round(z_max, 1)
z_min = round(z_min, 1)
# workspace constraint
test_world.add_constraint(
                name='room_workspace',
                type='workspace',
                frame='world',
                geometry_type='area',
                points=[[x_min, y_min], [x_min, y_max], [x_max, y_min], [x_max, y_max]])

input_x = [0.0, 0.0,-2.0,-4.2,-5.0,-4.0,-1.3, 0.0, 0.0,-1.0,-3.0,-6.0,-6.7,-6.0,-3.0,-2.0, 0.0, 2.0, 2.5, 6.0, 7.0, 6.5, 6.0, 4.0, 2.7, 2.0, 0.0, 0.0]
input_y = [0.0, 2.0, 3.2, 3.2, 2.0, 1.0, 1.0, 0.0,-1.0,-1.8,-2.0,-2.2,-2.9,-3.6,-3.5,-2.0,-1.5,-1.5,-3.7,-4.0,-3.7,-3.5,-3.9,-4.0,-3.7,-2.0,-1.0, 0.0]

x, y, yaw, k, travel = pycubicspline.calc_2d_spline_interpolation(input_x, input_y, num=200)

grids = og.calculate_grids(x, y)
constraint = test_world.constraints.get('room_workspace')
for each in grids:
    constraint.add_hole(
        type='area',
        points=each)
for each in ws.get_cuts():
    constraint.add_hole(
        type='area',
        points=each)
print("end workspace")

print("start ceiling")
# add ground objects
test_world.add_constraint(
    name='tangent_to_ceiling_plane',
    type='tangent',
    frame='world',
    reference=dict(
        type='plane',
        args=dict(
            origin=[0, 0, z_max - 1.0],
            normal=[0, 0, 1]
        )
    )
)
test_world.add_asset(
    tag='dyn_box',
    description=dict(
        type='box',
        args=dict(
            size=[0.8, 0.8, 0.8],
            name='cuboid',
            color='xkcd'
        )
    )
)
test_world.add_asset(
    tag='static_cylinder',
    description=dict(
        type='cylinder',
        args=dict(
            length="0.8",
            radius="0.5",
            name='cylinder',
            color='xkcd'
        )
    )
)

placement_policy = dict(
    models=['dyn_box', 'static_cylinder'],
    config=[
        dict(
            dofs=['x', 'y'],
            tag='workspace',
            workspace='room_workspace'
        ),
        dict(
            dofs=['yaw'],
            tag='uniform',
            min=-3.141592653589793,
            max=3.141592653589793
        )
    ]
)
NUM_BOXES = 10
NUM_CYLINDER = 5
test_world.add_engine(
    tag='box_placement',
    engine_name='random_pose',
    models=['dyn_box', 'static_cylinder'],
    max_num=dict(
        dyn_box=NUM_BOXES,
        static_cylinder=NUM_CYLINDER),
    model_picker='random',
    no_collision=True,
    policies=[placement_policy],
    constraints=[
        dict(
            model='dyn_box',
            constraint='tangent_to_ceiling_plane'),
        dict(
            model='static_cylinder',
            constraint='tangent_to_ceiling_plane')
    ]
)
print("end ground")

print("start gen")
test_world.run_engines(attach_models=True)
print("generated")
test_world.export_world(output_dir=output_path,
                            filename=output_file,
                            with_default_ground_plane=False,
                            with_default_sun=False)
print("end")