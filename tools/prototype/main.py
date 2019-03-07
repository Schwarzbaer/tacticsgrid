# <Schwarzbaer> Wait... So making any arbitrary mesh collidable into is as
#   simple as setting an into collide mask on it? o.O
# <rdb> Schwarzbaer: yes!
# <rdb> It won't be very efficient compared to letting the egg loader set up the
#   collision geometry, though.
# <Schwarzbaer> Ah. ... So how do I make the loader do that? ^^
# <rdb> Schwarzbaer: add "<Collide> { Polyset keep descend }" under the <Group>
#   in egg file, or create a game property in Blender for the object called
#   "Collide" and set it to "Polyset keep descend"
# <rdb> If you are writing a preprocessing tool, you can probably use the
#   EggData API, and there's probably something in the API reference for
#   EggGroup to set the collide flags.
# <Schwarzbaer> Oh, I thought you'd meant something I could do during the
#   loading process itself.
# <rdb> Not without a small detour through the EggData API.
# <rdb> Or without using the ObjectType mechanism.


import sys
from panda3d.core import NodePath
from panda3d.core import LVector3
from panda3d.core import BitMask32
from panda3d.core import LineSegs
from panda3d.core import CollisionHandlerQueue
from panda3d.core import CollisionTraverser
from panda3d.core import CollisionNode
from panda3d.core import CollisionRay
from panda3d.core import CollisionSegment
from panda3d.core import RigidBodyCombiner
from direct.showbase.ShowBase import ShowBase
from direct.task.Task import Task


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


def find_stepping_points(navigation_mesh, spacing=0.5,
                         threshold=0.001, show_probes=False):
    # spacing: Distance between adjacent points
    # threshold: Maximum distance at which collisions will be considered
    #   to be the same point, and filtered out
    # TODO: Switches to make collision tests sequential, to save memory at
    #   the cost of time.

    # TODO: This should be attached to navigation_mesh. However, that spams
    #   this error why?
    #   :mathutil(warning): BoundingLine::contains_geometric() called with BoundingLine
    #collision_root = navigation_mesh.attach_new_node(CollisionNode('ray_grid'))
    collision_root = s.render.attach_new_node(CollisionNode('ray_grid'))
    collision_root.node().setFromCollideMask(BitMask32.bit(1))
    if show_probes:
        collision_root.show()
    collision_queue = CollisionHandlerQueue()
    collision_traverser = CollisionTraverser('stepping point traverser')
    collision_traverser.add_collider(collision_root, collision_queue)
    if show_probes:
        collision_traverser.show_collisions(navigation_mesh)

    def step_values(low_bound, high_bound, spacing):
        num_steps = int((high_bound - low_bound) / spacing + 1)
        step_values = (low_bound + i * spacing for i in range(0, num_steps))
        return step_values

    print("Creating collision rays...")
    num_rays = 0
    (x_low, y_low, z_low), (x_high, y_high, z_high) = navigation_mesh.get_tight_bounds()
    for x in step_values(x_low, x_high, spacing):
        for y in step_values(y_low, y_high, spacing):
            ray = CollisionRay((x, y, z_high), (0, 0, -1))
            collision_root.node().add_solid(ray)
            num_rays += 1
    print("Created {} collision rays".format(num_rays))

    collision_traverser.traverse(navigation_mesh)
    filtered_points = []
    rejects = 0
    for entry in collision_queue.get_entries():
        pos = entry.get_surface_point(navigation_mesh)
        if not any(((pos - fp).length() < threshold for fp in filtered_points)):
            filtered_points.append(pos)
        else:
            rejects += 1
    print("{} stepping points, rejected {} as duplicates".format(
        len(filtered_points), rejects,
    ))

    collision_root.remove_node()
    return filtered_points


def stepping_debug_mesh(stepping_points, size=0.01):
    combiner = RigidBodyCombiner("stepping debug mesh combiner")
    debug_mesh = NodePath(combiner)
    for stepping_point in stepping_points:
        debug_model = loader.load_model("octahedron")
        debug_model.reparent_to(debug_mesh)
        debug_model.set_pos(stepping_point)
        debug_model.set_scale(size)
    combiner.collect()
    debug_mesh.set_render_mode_wireframe()
    return debug_mesh


def find_view_points(stepping_points, view_height=1.6):
    # view_height: Height above floor at which visibility tests are made
    view_offset = LVector3(0, 0, view_height)
    view_points = [p + view_offset for p in stepping_points]
    return view_points


def view_point_debug_mesh(view_points, size=0.01):
    return stepping_debug_mesh(view_points, size=size)


def test_visibility(view_points, obstruction_model, show_probes=False):
    # TODO: Attach to obstruction model instead
    collision_root = s.render.attach_new_node(CollisionNode('visibility_net'))
    collision_root.node().setFromCollideMask(BitMask32.bit(1))
    if show_probes:
        collision_root.show()
    collision_queue = CollisionHandlerQueue()
    collision_traverser = CollisionTraverser('visibility traverser')
    collision_traverser.add_collider(collision_root, collision_queue)
    if show_probes:
        collision_traverser.show_collisions(obstruction_model)

    # TODO: Eww.
    view_p = view_points
    idx_to_segment = {}
    segment_to_idx = {}
    for idx, point in enumerate(view_p):
        segment = CollisionSegment((0, 0, 0), point)
        collision_root.node().add_solid(segment)
        idx_to_segment[idx] = segment
        segment_to_idx[segment] = idx

    visibility = {}
    for idx_from, from_point in enumerate(view_p):
        for idx_to, to_point in enumerate(view_p):
            idx_to_segment[idx_to].set_point_a(from_point)
        collision_traverser.traverse(obstruction_model)
        occlusion = [segment_to_idx[p.get_from()]
                     for p in collision_queue.get_entries()]
        visibility[idx_from] = [i
                                for i in range(len(view_p))
                                if i not in occlusion]
        print(idx_from, len(visibility[idx_from]))

    collision_root.remove_node()
    return visibility


def visibility_debug_mesh(stepping_points, visibility, thickness=1):
    ls = LineSegs()
    ls.set_thickness(thickness)
    ls.set_color(1, 1, 0)
    for stepping_idx, stepping_point in enumerate(stepping_points):
        for visible_point in visibility[stepping_idx]:
            ls.move_to(stepping_point)
            ls.draw_to(stepping_points[visible_point])
    debug_mesh = ls.create()
    return NodePath(debug_mesh)


print("Finding stepping/view points")
#model.show()
#model.hide()
navmesh.hide()
level.show()
stepping_points = find_stepping_points(navmesh, spacing=2.0)
#debug_mesh = stepping_debug_mesh(stepping_points, size=0.05)
#debug_mesh.reparent_to(s.render)
#debug_mesh.show()
view_points = find_view_points(stepping_points, view_height=1.6)
debug_mesh = view_point_debug_mesh(view_points, size=0.05)
debug_mesh.reparent_to(s.render)
#debug_mesh.hide()
debug_mesh.show()
print("Checking mutual visibility")
visibility = test_visibility(view_points, level)
vis_debug_mesh = visibility_debug_mesh(view_points, visibility,
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
