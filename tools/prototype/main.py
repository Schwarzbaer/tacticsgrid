import sys

from panda3d.core import BitMask32
from panda3d.core import NodePath
from panda3d.core import Filename
from panda3d.core import Loader
from panda3d.core import LoaderOptions

from panda3d.egg import EggData
from panda3d.egg import load_egg_data

from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task

from tacticsgrid import grid


# Load (PandaLoader)
#loader = Loader.getGlobalPtr()
#node = loader.loadSync(Filename("navmeshthingy.egg"), LoaderOptions())
#model = NodePath(node)

# Load (EggData)
egg_data = EggData()
egg_data.read(Filename("navmeshthingy.egg"))
node = load_egg_data(egg_data)
model = NodePath(node)


# Separate navmesh and level geometry
navmesh = NodePath("navmesh")
level = NodePath("level")
for geomnode in model.node().get_children():
    if geomnode.get_name() == "Navmesh":
        navmesh.node().add_child(geomnode)
    else:
        level.node().add_child(geomnode)


# TODO: This should probably be in the generate_cover_data function, and not set
#   the From mask, but the latter is necessary since setFromCollideMask() does
#   not work recursively on this object.
navmesh.setCollideMask(BitMask32.bit(1))
level.setCollideMask(BitMask32.bit(1))


print("Finding stepping/view points")
stepping_points = grid.find_stepping_points(navmesh, spacing=2.0)
view_points = grid.find_view_points(stepping_points, view_height=1.6)
print("Checking mutual visibility")
visibility = grid.test_visibility(view_points, level)
print("Calculations done")


# Visualization


s = ShowBase()
s.accept("escape", sys.exit)


navmesh.hide()
level.reparent_to(s.render)
level.show()


step_debug_mesh = grid.view_point_debug_mesh(view_points, size=0.05)
step_debug_mesh.reparent_to(level)


vis_debug_mesh = grid.visibility_debug_mesh(view_points, visibility,
                                            thickness=1)
vis_debug_mesh.reparent_to(level)


low_bound, high_bound = level.get_tight_bounds()
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
