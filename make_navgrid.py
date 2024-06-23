from collections import defaultdict
from itertools import product

from panda3d.core import Vec2
from panda3d.core import Vec3
from panda3d.core import NodePath
from panda3d.core import CollisionTraverser
from panda3d.core import CollisionHandlerQueue
from panda3d.core import CollisionNode
from panda3d.core import CollisionRay
from panda3d.core import CollisionSphere
from panda3d.core import CollisionSegment
from panda3d.core import LineSegs

from direct.showbase.ShowBase import ShowBase

from optimizer import optimize_collisions


class DebugVisualization:
    def __init__(self, level):
        self.level = level
        self.debug_vis = []
        self.adj_lines = None

    def update(self, navgrid, adjacency):
        # Grid points
        # Adjust number of visual debug indicators
        if len(self.debug_vis) > len(navgrid):
            for dv in self.debug_vis[len(navgrid):]:
                dv.remove_node()
            self.debug_vis = self.debug_vis[0:len(navgrid)]
        elif len(self.debug_vis) < len(navgrid):
            for _ in range(len(navgrid) - len(self.debug_vis)):
                np = base.loader.load_model("models/smiley")
                np.set_scale(0.1)
                np.reparent_to(self.level)
                self.debug_vis.append(np)
        # Adjust positions
        for dv, (_, _, pos) in zip(self.debug_vis, navgrid):
            dv.set_pos(pos)

        # Adjacencies
        if self.adj_lines is not None:
            self.adj_lines.remove()
        offset = Vec3(0, 0, 0.1)
        ls = LineSegs()
        for from_idx, to_idx, cost in adjacency:
            _, _, from_coord = navgrid[from_idx]
            _, _, to_coord = navgrid[to_idx]
            ls.set_color(cost - 1.0, 2.0 - cost, 0)
            ls.move_to(from_coord + offset)
            ls.set_color(cost - 1.0, 2.0 - cost, 0)
            ls.draw_to(to_coord + offset)
        self.adj_lines = NodePath(ls.create())
        self.adj_lines.reparent_to(self.level)
        #import pdb; pdb.set_trace()
        
    def destroy(self):
        for dv in self.debug_vis:
            dv.remove_node()
        self.debug_vis = []


def find_footfalls(level, origin, x_interval, y_interval):
    traverser = CollisionTraverser('first point finder')
    ray = CollisionRay(0, 0, 0, 0, 0, -1)
    ray_node = CollisionNode('ray')
    ray_node.add_solid(ray)
    ray_np = NodePath(ray_node)
    ray_np.reparent_to(level)
    queue = CollisionHandlerQueue()
    traverser.add_collider(ray_np, queue)

    def scan_coord(origin, x, y):
        ray_np.set_pos(origin)
        traverser.traverse(level)
        # We collect points in a set to weed out duplicates.
        navpoints = set(
            (x, y, entry.get_surface_point(level))
            for entry in queue.entries
        )
        return list(navpoints)

    navgrid = []
    x_idx = 0
    x = x_interval[0]
    while x <= x_interval[1]:
        y_idx = 0
        y = y_interval[0]
        while y <= y_interval[1]:
            coord = Vec3(
                origin.x + x,
                origin.y + y,
                origin.z,
            )
            new_places = scan_coord(coord, x_idx, y_idx)
            navgrid += new_places
            y_idx += 1
            y += y_interval[2]
        x_idx += 1
        x += x_interval[2]

    ray_np.remove_node()
    return navgrid


def filter_for_standability(level, navgrid):
    traverser = CollisionTraverser('standability tester')
    sphere = CollisionSphere(0, 0, 1, 0.8)
    sphere_node = CollisionNode('sphere')
    sphere_node.add_solid(sphere)
    sphere_np = NodePath(sphere_node)
    sphere_np.reparent_to(level)
    queue = CollisionHandlerQueue()
    traverser.add_collider(sphere_np, queue)
    def is_standable(pos):
        sphere_np.set_pos(pos)
        traverser.traverse(level)
        return not bool(queue.get_num_entries())
    navgrid = [(x, y, pos) for x, y, pos in navgrid if is_standable(pos)]

    sphere_np.remove_node()
    return navgrid


# The 8 neighborhood
neighbor_coords = list(product(range(-1, 2), range(-1, 2)))
neighbor_coords.remove((0, 0))


class TerrainTraverser:
    def __init__(self, level):
        self.level = level

        self.traverser = CollisionTraverser('first point finder')
        self.segment = CollisionSegment(0, 0, 0, 1, 0, 0)
        segment_node = CollisionNode('segment')
        segment_node.add_solid(self.segment)
        self.segment_np = NodePath(segment_node)
        self.segment_np.reparent_to(self.level)
        self.queue = CollisionHandlerQueue()
        self.traverser.add_collider(self.segment_np, self.queue)

    def is_traversible(self, from_coord, to_coord):
        dz = to_coord.z - from_coord.z
        dxy = (Vec2(to_coord.x, to_coord.y) - Vec2(from_coord.x, from_coord.y)).length()
        # Blot out the 45° cone above the from coord
        if dz > dxy:  
            return False

        # Collision check a little off the ground
        offset = Vec3(0, 0, 0.5)
        self.segment.set_point_a(from_coord + offset)
        self.segment.set_point_b(to_coord + offset)
        self.traverser.traverse(self.level)

        if self.queue.get_num_entries():  # If 1 or more, abort.
            return False

        # Yup, we can traverse, but at what cost?
        cost = dxy
        if dz > 0:  # Falling is considered free, climbing costs extra.
            cost = cost * (1 + dz / dxy)  # LERP factor between 1 and 2.
        return cost

    def remove(self):
        self.segment_np.remove_node()


def determine_adjacenjy(level, navgrid):
    idx = 0
    coords = set()  # (x, y)
    by_coords = defaultdict(list)  # (x, y): [(idx, pos), ...]
    adjacency = []
    tt = TerrainTraverser(level)
    for x, y, pos in navgrid:
        coords.add((x, y))
        by_coords[(x,y)].append((idx, pos))
        idx += 1
    progress = 0
    for x, y in coords:
        for from_idx, from_pos in by_coords[(x, y)]:
            for dx, dy in neighbor_coords:
                nx, ny = x + dx, y + dy
                if (nx, ny) in by_coords:
                    for to_idx, to_pos in by_coords[(nx, ny)]:
                        cost = tt.is_traversible(from_pos, to_pos)
                        if cost:
                            adjacency.append((from_idx, to_idx, cost))
        
    tt.remove()
    return adjacency


if __name__=='__main__':
    ShowBase()
    base.accept('escape', base.task_mgr.stop)
    cam_focus = Vec3(0, 0, 0)
    cam_offset = Vec3(-40, -40, 40)
    base.cam.set_pos(cam_focus + cam_offset)
    base.cam.look_at(cam_focus)

    level = base.loader.load_model("level.bam")
    level.set_collide_mask(1)
    optimize_collisions(level, convert_geometry=True)
    level.reparent_to(base.render)

    # origin = Vec3(0, 0, 10)
    # x_interval = (-15, 15, 0.5)
    # y_interval = (-15, 15, 0.5)
    bottom, top = level.get_tight_bounds()
    origin = Vec3(0, 0, top.z + 10)
    x_interval = (bottom.x, top.x, 0.5)
    y_interval = (bottom.y, top.y, 0.5)
    print("Finding footfalls")
    navgrid = find_footfalls(
        level,
        origin,
        x_interval,
        y_interval,
    )
    print("Filtering for standability")
    navgrid = filter_for_standability(level, navgrid)
    print("Determining adjacency")
    adjacency = determine_adjacenjy(level, navgrid)
    print("Creating debug visualization")
    DebugVisualization(level).update(navgrid, adjacency)
    base.run()
