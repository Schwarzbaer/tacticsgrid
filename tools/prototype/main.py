import sys

from panda3d.core import BitMask32

from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task

from tacticsgrid import grid


s = ShowBase()
s.accept("escape", sys.exit)


#model = base.loader.load_model("sphere")
model = base.loader.load_model("navmeshthingy.egg")
#model.reparent_to(s.render)
navmesh = s.render.attach_new_node("navmesh")
level = s.render.attach_new_node("level")

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


low_bound, high_bound = level.get_tight_bounds()
bound_size = (high_bound - low_bound).length()
print("map size: {}".format(high_bound - low_bound))
center = low_bound + (high_bound - low_bound) * 0.5
cam_gimbal = s.render.attach_new_node("cam gimbal")
cam_gimbal.set_pos(center)
s.cam.reparent_to(cam_gimbal)
s.cam.set_pos(0, -bound_size * 2, bound_size * 0.75)
s.cam.look_at(0, 0, 0)


print("Finding stepping/view points")
#model.show()
#model.hide()
navmesh.hide()
level.show()
stepping_points = grid.find_stepping_points(navmesh, spacing=2.0)
#debug_mesh = stepping_debug_mesh(stepping_points, size=0.05)
#debug_mesh.reparent_to(s.render)
#debug_mesh.show()
view_points = grid.find_view_points(stepping_points, view_height=1.6)
debug_mesh = grid.view_point_debug_mesh(view_points, size=0.05)
debug_mesh.reparent_to(s.render)
#debug_mesh.hide()
debug_mesh.show()
print("Checking mutual visibility")
visibility = grid.test_visibility(view_points, level)
vis_debug_mesh = grid.visibility_debug_mesh(view_points, visibility,
                                            thickness=1)
vis_debug_mesh.reparent_to(s.render)
#vis_debug_mesh.hide()
vis_debug_mesh.show()
print("Done")


def rotate_camera(task):
    cam_gimbal.set_h(task.time*(360/5))
    return Task.cont


s.taskMgr.add(rotate_camera, "rotate camera")
s.run()
