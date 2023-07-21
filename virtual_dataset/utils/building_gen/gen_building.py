from pcg_gazebo.parsers.sdf import create_sdf_element
from pcg_gazebo.parsers.sdf_config import create_sdf_config_element
from buiding import Building
from sdf_util import visual_setting, collision_setting
import os

fp_path = "floorplans/room1.txt"
b = Building(fp_path)
b.read_floorplan()
b.create_model()
walls = b.get_model()
beams = b.get_beams()

model = create_sdf_element('model')
model.name = 'room1_mirror_room'
model.static = True
model.self_collide = False
model.allow_auto_disable = True

path = "/home/greg/.gazebo/models/"
save_path = path + model.name

for i in range(len(walls)):

    wall_name = "Wall_" + str(i)
    temp_link = create_sdf_element("link")
    temp_link.name = wall_name
    temp_link.gravity = False
    temp_link.pose = [walls[i][0][0], walls[i][0][1], 0, 0, 0, 0]
    # print(temp_link)

    visual = visual_setting(wall_name, walls[i])
    temp_link.add_visual(visual.name, visual)

    collision = collision_setting(wall_name, walls[i])
    temp_link.add_collision(collision.name, collision)

    model.add_link(temp_link.name, temp_link)

for i in range(len(beams)):

    beam_name = "Beam_" + str(i)
    temp_link = create_sdf_element("link")
    temp_link.name = beam_name
    temp_link.gravity = False
    temp_link.pose = [beams[i][0][0], beams[i][0][1], b.height/2 - beams[i][3]/2, 0, 0, 0]
    # print(temp_link)

    visual = visual_setting(beam_name, beams[i])
    temp_link.add_visual(visual.name, visual)

    collision = collision_setting(beam_name, beams[i])
    temp_link.add_collision(collision.name, collision)

    model.add_link(temp_link.name, temp_link)

ceiling = create_sdf_element('model')
ceiling.name = model.name + '_ceiling'
ceiling.static = True
ceiling.self_collide = False
ceiling.allow_auto_disable = True
ceiling_name = "Ceiling"
temp_link = create_sdf_element("link")
temp_link.name = ceiling_name
temp_link.gravity = False
temp_link.pose = [0, 0, 0, 0, 0, 0]
visual = visual_setting(ceiling_name, [0, (b.size[1])/(b.wall_thickness / 0.2)+2, (b.size[0])/(b.wall_thickness / 0.2)+2, 0.2])
temp_link.add_visual(visual.name, visual)
collision = collision_setting(ceiling_name, [0, (b.size[1])/(b.wall_thickness / 0.2)+2, (b.size[0])/(b.wall_thickness / 0.2)+2, 0.2])
temp_link.add_collision(collision.name, collision)
ceiling.add_link(temp_link.name, temp_link)

sdf = create_sdf_element('sdf')
sdf.reset('model')
sdf.version = model.sdf_version
sdf.add_model(model.name, model)

if os.path.exists(save_path):
    print("model with same name exists")
else:
    os.mkdir(save_path)

    manifest = create_sdf_config_element('model')
    manifest.name = model.name
    manifest.version = 1.0
    manifest.description = ''
    manifest.add_sdf()
    manifest.sdfs[0].version = model.sdf_version
    manifest.sdfs[0].value = "model.sdf"
    manifest.add_author()
    manifest.authors[0].name = ''
    manifest.authors[0].email = ''

    sdf.export_xml(os.path.join(save_path, "model.sdf"))
    manifest.export_xml(os.path.join(save_path, "model.config"))

    print("export success")

save_path_ceiling = path + ceiling.name

sdf_ceiling = create_sdf_element('sdf')
sdf_ceiling.reset('model')
sdf_ceiling.version = ceiling.sdf_version
sdf_ceiling.add_model(ceiling.name, ceiling)

if os.path.exists(save_path_ceiling):
    print("ceiling model with same name exists")
else:
    os.mkdir(save_path_ceiling)

    manifest = create_sdf_config_element('model')
    manifest.name = model.name
    manifest.version = 1.0
    manifest.description = ''
    manifest.add_sdf()
    manifest.sdfs[0].version = ceiling.sdf_version
    manifest.sdfs[0].value = "model.sdf"
    manifest.add_author()
    manifest.authors[0].name = ''
    manifest.authors[0].email = ''

    sdf_ceiling.export_xml(os.path.join(save_path_ceiling, "model.sdf"))
    manifest.export_xml(os.path.join(save_path_ceiling, "model.config"))

    print("ceiling export success")
# print(model)
print("end")