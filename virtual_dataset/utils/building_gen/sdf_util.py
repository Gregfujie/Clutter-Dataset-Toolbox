from pcg_gazebo.parsers.sdf import create_sdf_element


def visual_setting(wall_name, wall):
    temp_visual = create_sdf_element("visual")
    temp_visual.reset(with_optional_elements=True)
    temp_visual.name = wall_name + "_visual"
    temp_visual.geometry.box = dict(size=[wall[1], wall[2], wall[3]])
    temp_visual.material.ambient = [1, 1, 1, 1]
    temp_visual.material.children.pop("lighting")
    temp_visual.material.children.pop("diffuse")
    temp_visual.material.children.pop("specular")
    temp_visual.material.children.pop("emissive")
    return temp_visual


def collision_setting(wall_name, wall):
    temp_collision = create_sdf_element("collision")
    temp_collision.reset(with_optional_elements=True)
    temp_collision.name = wall_name + "_collision"
    temp_collision.geometry.box = dict(size=[wall[1], wall[2], wall[3]])
    temp_collision.surface.contact.collide_bitmask = 1
    temp_collision.surface.contact.children.pop("category_bitmask")
    temp_collision.surface.contact.children.pop("poissons_ratio")
    temp_collision.surface.contact.children.pop("elastic_modulus")
    return temp_collision


if __name__ == '__main__':
    a = visual_setting("test", [0, 0, 0, 0])
    b = collision_setting("test", [0, 0, 0, 0])
    print(a)
    print(b)
    print("end")