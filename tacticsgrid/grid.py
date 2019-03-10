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


def find_stepping_points(navigation_mesh, spacing=0.5,
                         threshold=0.001, show_probes=False):
    # spacing: Distance between adjacent points
    # threshold: Maximum distance at which collisions will be considered
    #   to be the same point, and filtered out
    # TODO: Switches to make collision tests sequential, to save memory at
    #   the cost of time.

    collision_root = navigation_mesh.attach_new_node(CollisionNode('ray_grid'))
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
    collision_root = obstruction_model.attach_new_node(CollisionNode('visibility_net'))
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
