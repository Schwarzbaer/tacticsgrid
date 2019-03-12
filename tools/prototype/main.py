import sys

from panda3d.core import BitMask32
from panda3d.core import NodePath
from panda3d.core import Filename
from panda3d.core import Loader
from panda3d.core import LoaderOptions

from panda3d.egg import EggData
from panda3d.egg import EggGroup
from panda3d.egg import save_egg_data
from panda3d.egg import load_egg_data

from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task

from tacticsgrid import grid


# Load (PandaLoader)
loader = Loader.getGlobalPtr()
node = loader.loadSync(Filename("navmeshthingy.egg"), LoaderOptions())
model = NodePath(node)


# Separate navmesh and level geometry
navmesh_raw = NodePath("navmesh")
level_raw = NodePath("level")
for geomnode in model.node().get_children():
    if geomnode.get_name() == "Navmesh":
        navmesh_raw.node().add_child(geomnode)
    else:
        level_raw.node().add_child(geomnode)


# Create efficiently collidable models
def add_collider_polyset(model):
    egg_model = EggData()
    save_egg_data(egg_model, model.node())
    for child in egg_model.children:
        if child.get_class_type().compare_to(EggGroup) == 0:
            print(child.get_name())
            child.set_cs_type(EggGroup.CST_polyset)
            child.set_collide_flags(EggGroup.CF_keep | EggGroup.CF_descend)
    # egg_data.writeEgg(Filename("navtest2.egg"))
    new_model_node = load_egg_data(egg_model)
    new_model = NodePath(new_model_node)
    return new_model


level = add_collider_polyset(level_raw)
navmesh = add_collider_polyset(navmesh_raw)
#navmesh.setCollideMask(BitMask32.bit(1))
#level.setCollideMask(BitMask32.bit(1))


# Load (EggData)
# egg_data = EggData()
# egg_data.read(Filename("navmeshthingy.egg"))
# egg_data.writeEgg(Filename("navtest1.egg"))
# egg_children = egg_data.get_children()


print("Finding stepping/view points")
stepping_points = grid.find_stepping_points(navmesh, spacing=1)
view_points = grid.find_view_points(stepping_points, view_height=1.6)
print("Checking mutual visibility")
visibility = grid.test_visibility(view_points, level)
print("Calculations done")


# Visualization
s = ShowBase()
s.accept("escape", sys.exit)


# navmesh_raw.hide()
level_raw.reparent_to(s.render)
# level.show()


step_debug_mesh = grid.view_point_debug_mesh(view_points, size=0.05)
step_debug_mesh.reparent_to(level_raw)


vis_debug_mesh = grid.visibility_debug_mesh(view_points, visibility,
                                            thickness=1)
vis_debug_mesh.reparent_to(level_raw)


low_bound, high_bound = level_raw.get_tight_bounds()
bound_size = (high_bound - low_bound).length()
print("map size: {}".format(high_bound - low_bound))
center = low_bound + (high_bound - low_bound) * 0.5
cam_gimbal = s.render.attach_new_node("cam gimbal")
cam_gimbal.set_pos(center)
s.cam.reparent_to(cam_gimbal)
s.cam.set_pos(0, -bound_size * 2, bound_size * 0.75)
s.cam.look_at(0, 0, 0)


def rotate_camera(task):
    cam_gimbal.set_h(task.time*(360/5))
    return Task.cont


s.taskMgr.add(rotate_camera, "rotate camera")
s.run()
