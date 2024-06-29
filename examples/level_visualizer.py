from random import choice

from panda3d.core import Vec3

from direct.showbase.ShowBase import ShowBase

from pychology.simple_search.a_star import search
from pychology.simple_search.a_star import NoPath

from tacticsgrid.navgrid import scan_level
from tacticsgrid.navgrid import nearest_node
from tacticsgrid.navgrid import DebugVisualization
from tacticsgrid.optimizer import optimize_collisions


if __name__=='__main__':
    ShowBase()
    base.accept('escape', base.task_mgr.stop)
    base.set_frame_rate_meter(True)

    level = base.loader.load_model("level.bam")
    level.set_collide_mask(1)
    optimize_collisions(level, convert_geometry=True)
    level.reparent_to(base.render)

    bottom, top = level.get_tight_bounds()
    cam_focus = bottom + top * 0.5
    cam_offset = Vec3(-1, -1, 1) * (top - bottom).length()
    base.cam.set_pos(cam_focus + cam_offset)
    base.cam.look_at(cam_focus)

    print("Starting level scan...")
    navgrid, adjacency = scan_level(level, 1.0)
    print(f"{len(navgrid)} nodes.")
    #print("converting to wezu standard")
    #wezu_navgrid = to_wezu(navgrid, adjacency)
    #from pprint import pprint
    #pprint(wezu_navgrid)
    #import pdb; pdb.set_trace()

    print("Creating debug visualization")
    #DebugVisualization(level).update(navgrid, adjacency)
    
    dv = DebugVisualization(level)
    click_path = [None, None]
    def neighbors(idx):
        return [(i, c) for i, c in adjacency[idx].items()]
    def euclidean_distance(from_idx, to_idx):
        return (navgrid[from_idx][2] - navgrid[to_idx][2]).length()
    def update_path(from_idx, to_idx):
        try:
            path = search(neighbors, from_idx, to_idx, euclidean_distance)
            #print(f"Path from {from_idx} to {to_idx}")
            dv.show_path(path, navgrid)
        except NoPath:
            #print(f"No path from {from_idx} to {to_idx}")
            pass
    def update_click_path():
        click_path.append(choice(list(adjacency.keys())))
        click_path.pop(0)
        if click_path[0] is not None:
            update_path(click_path[0], click_path[1])
        print(click_path)
    def random_path(task):
        #from_idx = nearest_node(navgrid, Vec3(8, 0, 3))
        #to_idx = nearest_node(navgrid, Vec3(-4, 0, 3))
        #import pdb; pdb.set_trace()
        from_idx = choice(list(adjacency.keys()))
        to_idx = choice(list(adjacency.keys()))
        update_path(from_idx, to_idx)
        return task.again
    base.accept("mouse1", update_click_path)
    #base.task_mgr.do_method_later(1, random_path, 'update path')
    #base.task_mgr.add(random_path)
    base.run()
