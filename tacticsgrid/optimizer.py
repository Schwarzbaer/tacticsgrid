from panda3d.core import *
from array import array


def optimize_collisions(np, *, convert_geometry=False, ignore_z=False, preserve_name=True, preserve_tags=True, track_progress=None):
    """Organizes all "into" collision nodes below this level into an octree or
    quadtree (actually an AABB tree, but never mind that) for greatly speeding
    up collisions.

    If convert_geometry is True, also converts all GeomNodes (unless you set
    their "into" masks to zero explicitly) into CollisionNodes.

    If ignore_z is True, there is no splitting along the Z axis, which may be
    better if you are dealing exclusively with flat meshes.

    If preserve_name is True, indicates you find the name of the CollisionNode
    important and want to keep it (but there'll be less merging).

    If preserve_tags is True, indicates you find the tags of the CollisionNode
    important and want to keep them.

    If track_progress is True, shows a progress bar.  Can alternatively be a
    function.
    """

    max_leaf = 4 if ignore_z else 8

    if __debug__ and track_progress is True:
        try:
            from pip._vendor.rich.progress import track as track_progress
        except ImportError:
            from rich.progress import track as track_progress
        track = lambda x, desc: track_progress(x, desc, transient=True)
    elif track_progress and track_progress is not True:
        # Custom function
        track = lambda x, desc: track_progress(x, desc)
    else:
        track = lambda x, desc: x

    # Collect solids, calculating min/max origin while we do so
    min = None
    max = None
    solids = []
    node = np.node()
    if node.is_of_type(CollisionNode.get_class_type()):
        transform = TransformState.make_identity()
        for solid in node.solids:
            origin = solid.get_collision_origin()
            if min is None:
                min = origin
                max = origin
            else:
                min = min.fmin(origin)
                max = max.fmax(origin)
            name = node.name if preserve_name else ''
            tags = tuple(sorted(node.tags.items())) if preserve_tags else ()
            solids.append((solid, transform, node.into_collide_mask, name, tags, origin))
        if node.from_collide_mask != 0:
            node.into_collide_mask = 0
        else:
            node.clear_solids()
    del node

    # Collect solids on any child nodes
    for child in track(np.find_all_matches('**/+CollisionNode'), "Collecting solids..."):
        child_node = child.node()
        if child_node.into_collide_mask == 0:
            continue

        for solid in child_node.solids:
            transform = child.get_transform(np)
            origin = solid.get_collision_origin()
            origin = transform.get_mat().xform_point(origin)
            if min is None:
                min = origin
                max = origin
            else:
                min = min.fmin(origin)
                max = max.fmax(origin)
            name = child_node.name if preserve_name else ''
            tags = tuple(sorted(child_node.tags.items())) if preserve_tags else ()
            solids.append((solid, transform, child_node.into_collide_mask, name, tags, origin))

        if child_node.from_collide_mask != 0:
            child_node.into_collide_mask = 0
        elif child_node.get_num_children() == 0:
            child.remove_node()
        else:
            child_node.clear_solids()

    # Collect GeomNodes with into collide masks
    if convert_geometry:
        transform = TransformState.make_identity()

        for child in track(np.find_all_matches('**/+GeomNode'), "Converting meshes..."):
            child_node = child.node()
            if child_node.into_collide_mask == 0:
                continue

            name = child_node.name if preserve_name else ''
            tags = tuple(sorted(child_node.tags.items())) if preserve_tags else ()
            mask = child_node.into_collide_mask

            for geom in child_node.get_geoms():
                if geom.primitive_type != Geom.PT_polygons:
                    continue

                vertex = GeomVertexReader(geom.get_vertex_data(), 'vertex')

                geom = geom.decompose()
                geom.transform_vertices(transform.get_mat())

                for prim in geom.get_primitives():
                    for i in range(0, prim.get_num_vertices(), 3):
                        vertex.set_row(prim.get_vertex(i))
                        v1 = vertex.get_data3()
                        vertex.set_row(prim.get_vertex(i + 1))
                        v2 = vertex.get_data3()
                        vertex.set_row(prim.get_vertex(i + 2))
                        v3 = vertex.get_data3()
                        origin = (v1 + v2 + v3) * (1.0 / 3)
                        if min is None:
                            min = origin
                            max = origin
                        else:
                            min = min.fmin(origin)
                            max = max.fmax(origin)
                        solid = CollisionPolygon(v1, v2, v3)
                        solids.append((solid, transform, mask, name, tags, origin))

            child_node.into_collide_mask = 0

    if min is None:
        # Nothing to do!
        return

    # Compute morton code for each solid
    dims = max - min
    scale = VBase3(0xfffff / dims[0], 0xfffff / dims[1], 0xfffff / dims[2])
    if ignore_z:
        scale[2] = 0

    morton_solids = []
    for solid, transform, into_mask, name, tags, origin in track(solids, "Calculating codes..."):
        scaled = (origin - min)
        scaled.componentwise_mult(scale)
        code = get_morton_code(int(scaled[0]), int(scaled[1]), int(scaled[2]))
        morton_solids.append((code, transform, into_mask, name, tags, solid))

    # Sort by morton code, then transform + "into" mask + name + tags
    morton_solids.sort()

    it = iter(track(range(len(morton_solids)), "Building new tree..."))

    # This doesn't need to be recursive, but it's easier to understand/debug
    def build_level(parent, shift, offset, count):
        # code -> (offset, count) of child levels
        child_levels = [(0, 0)] * 8
        num_child_levels = 0

        shift -= 3

        for i in range(offset, offset + count):
            child_code = (morton_solids[i][0] >> shift) & 0b111
            child_offset, child_count = child_levels[child_code]
            if child_count == 0:
                child_offset = i
                num_child_levels += 1
            child_count += 1
            child_levels[child_code] = child_offset, child_count

        if num_child_levels == 0:
            return

        if count > 1:
            this_code = (morton_solids[offset][0] >> (shift + 3))
            this_code = bin(this_code)[2:].zfill(60 - (shift + 3))
            parent = parent.attach_new_node('col-' + this_code + 'x' * (shift + 3))
            parent.node().set_bounds_type(BoundingVolume.BT_box)
            parent.hide()

        for child_offset, child_count in child_levels:
            if count <= max_leaf or child_count == 1 or shift == 0:
                cnode = None
                cnode_tfrm = None
                cnode_mask = None
                cnode_name = None
                cnode_tags = None
                for i in range(child_offset, child_offset + child_count):
                    assert i == next(it)
                    code, tfrm, mask, name, tags, solid = morton_solids[i]
                    if tfrm != cnode_tfrm or mask != cnode_mask or name != cnode_name or tags != cnode_tags:
                        cnode = CollisionNode(name if preserve_name else 'col-' + bin(code)[2:].zfill(60))
                        cnode.into_collide_mask = mask
                        cnode.set_transform(tfrm)
                        for key, value in tags:
                            cnode.set_tag(key, value)
                        cnp = parent.attach_new_node(cnode)
                        cnode_tfrm = tfrm
                        cnode_mask = mask
                        cnode_name = name
                        cnode_tags = tags
                    cnode.add_solid(solid)
            elif child_count > 1:
                build_level(parent, shift, child_offset, child_count)

    build_level(np, 60, 0, len(morton_solids))

    for i in it:
        assert False


def get_morton_code(x, y, z):
    return morton1024[x & 0x3ff] \
        | (morton1024[y & 0x3ff] << 1) \
        | (morton1024[z & 0x3ff] << 2) \
        | (morton1024[x >> 10] << 30) \
        | (morton1024[y >> 10] << 31) \
        | (morton1024[z >> 10] << 32)


# Efficient lookup tables for Morton code calculation
morton1024 = array('I', (0x00000000,
0x00000001, 0x00000008, 0x00000009, 0x00000040, 0x00000041, 0x00000048,
0x00000049, 0x00000200, 0x00000201, 0x00000208, 0x00000209, 0x00000240,
0x00000241, 0x00000248, 0x00000249, 0x00001000, 0x00001001, 0x00001008,
0x00001009, 0x00001040, 0x00001041, 0x00001048, 0x00001049, 0x00001200,
0x00001201, 0x00001208, 0x00001209, 0x00001240, 0x00001241, 0x00001248,
0x00001249, 0x00008000, 0x00008001, 0x00008008, 0x00008009, 0x00008040,
0x00008041, 0x00008048, 0x00008049, 0x00008200, 0x00008201, 0x00008208,
0x00008209, 0x00008240, 0x00008241, 0x00008248, 0x00008249, 0x00009000,
0x00009001, 0x00009008, 0x00009009, 0x00009040, 0x00009041, 0x00009048,
0x00009049, 0x00009200, 0x00009201, 0x00009208, 0x00009209, 0x00009240,
0x00009241, 0x00009248, 0x00009249, 0x00040000, 0x00040001, 0x00040008,
0x00040009, 0x00040040, 0x00040041, 0x00040048, 0x00040049, 0x00040200,
0x00040201, 0x00040208, 0x00040209, 0x00040240, 0x00040241, 0x00040248,
0x00040249, 0x00041000, 0x00041001, 0x00041008, 0x00041009, 0x00041040,
0x00041041, 0x00041048, 0x00041049, 0x00041200, 0x00041201, 0x00041208,
0x00041209, 0x00041240, 0x00041241, 0x00041248, 0x00041249, 0x00048000,
0x00048001, 0x00048008, 0x00048009, 0x00048040, 0x00048041, 0x00048048,
0x00048049, 0x00048200, 0x00048201, 0x00048208, 0x00048209, 0x00048240,
0x00048241, 0x00048248, 0x00048249, 0x00049000, 0x00049001, 0x00049008,
0x00049009, 0x00049040, 0x00049041, 0x00049048, 0x00049049, 0x00049200,
0x00049201, 0x00049208, 0x00049209, 0x00049240, 0x00049241, 0x00049248,
0x00049249, 0x00200000, 0x00200001, 0x00200008, 0x00200009, 0x00200040,
0x00200041, 0x00200048, 0x00200049, 0x00200200, 0x00200201, 0x00200208,
0x00200209, 0x00200240, 0x00200241, 0x00200248, 0x00200249, 0x00201000,
0x00201001, 0x00201008, 0x00201009, 0x00201040, 0x00201041, 0x00201048,
0x00201049, 0x00201200, 0x00201201, 0x00201208, 0x00201209, 0x00201240,
0x00201241, 0x00201248, 0x00201249, 0x00208000, 0x00208001, 0x00208008,
0x00208009, 0x00208040, 0x00208041, 0x00208048, 0x00208049, 0x00208200,
0x00208201, 0x00208208, 0x00208209, 0x00208240, 0x00208241, 0x00208248,
0x00208249, 0x00209000, 0x00209001, 0x00209008, 0x00209009, 0x00209040,
0x00209041, 0x00209048, 0x00209049, 0x00209200, 0x00209201, 0x00209208,
0x00209209, 0x00209240, 0x00209241, 0x00209248, 0x00209249, 0x00240000,
0x00240001, 0x00240008, 0x00240009, 0x00240040, 0x00240041, 0x00240048,
0x00240049, 0x00240200, 0x00240201, 0x00240208, 0x00240209, 0x00240240,
0x00240241, 0x00240248, 0x00240249, 0x00241000, 0x00241001, 0x00241008,
0x00241009, 0x00241040, 0x00241041, 0x00241048, 0x00241049, 0x00241200,
0x00241201, 0x00241208, 0x00241209, 0x00241240, 0x00241241, 0x00241248,
0x00241249, 0x00248000, 0x00248001, 0x00248008, 0x00248009, 0x00248040,
0x00248041, 0x00248048, 0x00248049, 0x00248200, 0x00248201, 0x00248208,
0x00248209, 0x00248240, 0x00248241, 0x00248248, 0x00248249, 0x00249000,
0x00249001, 0x00249008, 0x00249009, 0x00249040, 0x00249041, 0x00249048,
0x00249049, 0x00249200, 0x00249201, 0x00249208, 0x00249209, 0x00249240,
0x00249241, 0x00249248, 0x00249249, 0x01000000, 0x01000001, 0x01000008,
0x01000009, 0x01000040, 0x01000041, 0x01000048, 0x01000049, 0x01000200,
0x01000201, 0x01000208, 0x01000209, 0x01000240, 0x01000241, 0x01000248,
0x01000249, 0x01001000, 0x01001001, 0x01001008, 0x01001009, 0x01001040,
0x01001041, 0x01001048, 0x01001049, 0x01001200, 0x01001201, 0x01001208,
0x01001209, 0x01001240, 0x01001241, 0x01001248, 0x01001249, 0x01008000,
0x01008001, 0x01008008, 0x01008009, 0x01008040, 0x01008041, 0x01008048,
0x01008049, 0x01008200, 0x01008201, 0x01008208, 0x01008209, 0x01008240,
0x01008241, 0x01008248, 0x01008249, 0x01009000, 0x01009001, 0x01009008,
0x01009009, 0x01009040, 0x01009041, 0x01009048, 0x01009049, 0x01009200,
0x01009201, 0x01009208, 0x01009209, 0x01009240, 0x01009241, 0x01009248,
0x01009249, 0x01040000, 0x01040001, 0x01040008, 0x01040009, 0x01040040,
0x01040041, 0x01040048, 0x01040049, 0x01040200, 0x01040201, 0x01040208,
0x01040209, 0x01040240, 0x01040241, 0x01040248, 0x01040249, 0x01041000,
0x01041001, 0x01041008, 0x01041009, 0x01041040, 0x01041041, 0x01041048,
0x01041049, 0x01041200, 0x01041201, 0x01041208, 0x01041209, 0x01041240,
0x01041241, 0x01041248, 0x01041249, 0x01048000, 0x01048001, 0x01048008,
0x01048009, 0x01048040, 0x01048041, 0x01048048, 0x01048049, 0x01048200,
0x01048201, 0x01048208, 0x01048209, 0x01048240, 0x01048241, 0x01048248,
0x01048249, 0x01049000, 0x01049001, 0x01049008, 0x01049009, 0x01049040,
0x01049041, 0x01049048, 0x01049049, 0x01049200, 0x01049201, 0x01049208,
0x01049209, 0x01049240, 0x01049241, 0x01049248, 0x01049249, 0x01200000,
0x01200001, 0x01200008, 0x01200009, 0x01200040, 0x01200041, 0x01200048,
0x01200049, 0x01200200, 0x01200201, 0x01200208, 0x01200209, 0x01200240,
0x01200241, 0x01200248, 0x01200249, 0x01201000, 0x01201001, 0x01201008,
0x01201009, 0x01201040, 0x01201041, 0x01201048, 0x01201049, 0x01201200,
0x01201201, 0x01201208, 0x01201209, 0x01201240, 0x01201241, 0x01201248,
0x01201249, 0x01208000, 0x01208001, 0x01208008, 0x01208009, 0x01208040,
0x01208041, 0x01208048, 0x01208049, 0x01208200, 0x01208201, 0x01208208,
0x01208209, 0x01208240, 0x01208241, 0x01208248, 0x01208249, 0x01209000,
0x01209001, 0x01209008, 0x01209009, 0x01209040, 0x01209041, 0x01209048,
0x01209049, 0x01209200, 0x01209201, 0x01209208, 0x01209209, 0x01209240,
0x01209241, 0x01209248, 0x01209249, 0x01240000, 0x01240001, 0x01240008,
0x01240009, 0x01240040, 0x01240041, 0x01240048, 0x01240049, 0x01240200,
0x01240201, 0x01240208, 0x01240209, 0x01240240, 0x01240241, 0x01240248,
0x01240249, 0x01241000, 0x01241001, 0x01241008, 0x01241009, 0x01241040,
0x01241041, 0x01241048, 0x01241049, 0x01241200, 0x01241201, 0x01241208,
0x01241209, 0x01241240, 0x01241241, 0x01241248, 0x01241249, 0x01248000,
0x01248001, 0x01248008, 0x01248009, 0x01248040, 0x01248041, 0x01248048,
0x01248049, 0x01248200, 0x01248201, 0x01248208, 0x01248209, 0x01248240,
0x01248241, 0x01248248, 0x01248249, 0x01249000, 0x01249001, 0x01249008,
0x01249009, 0x01249040, 0x01249041, 0x01249048, 0x01249049, 0x01249200,
0x01249201, 0x01249208, 0x01249209, 0x01249240, 0x01249241, 0x01249248,
0x01249249, 0x08000000, 0x08000001, 0x08000008, 0x08000009, 0x08000040,
0x08000041, 0x08000048, 0x08000049, 0x08000200, 0x08000201, 0x08000208,
0x08000209, 0x08000240, 0x08000241, 0x08000248, 0x08000249, 0x08001000,
0x08001001, 0x08001008, 0x08001009, 0x08001040, 0x08001041, 0x08001048,
0x08001049, 0x08001200, 0x08001201, 0x08001208, 0x08001209, 0x08001240,
0x08001241, 0x08001248, 0x08001249, 0x08008000, 0x08008001, 0x08008008,
0x08008009, 0x08008040, 0x08008041, 0x08008048, 0x08008049, 0x08008200,
0x08008201, 0x08008208, 0x08008209, 0x08008240, 0x08008241, 0x08008248,
0x08008249, 0x08009000, 0x08009001, 0x08009008, 0x08009009, 0x08009040,
0x08009041, 0x08009048, 0x08009049, 0x08009200, 0x08009201, 0x08009208,
0x08009209, 0x08009240, 0x08009241, 0x08009248, 0x08009249, 0x08040000,
0x08040001, 0x08040008, 0x08040009, 0x08040040, 0x08040041, 0x08040048,
0x08040049, 0x08040200, 0x08040201, 0x08040208, 0x08040209, 0x08040240,
0x08040241, 0x08040248, 0x08040249, 0x08041000, 0x08041001, 0x08041008,
0x08041009, 0x08041040, 0x08041041, 0x08041048, 0x08041049, 0x08041200,
0x08041201, 0x08041208, 0x08041209, 0x08041240, 0x08041241, 0x08041248,
0x08041249, 0x08048000, 0x08048001, 0x08048008, 0x08048009, 0x08048040,
0x08048041, 0x08048048, 0x08048049, 0x08048200, 0x08048201, 0x08048208,
0x08048209, 0x08048240, 0x08048241, 0x08048248, 0x08048249, 0x08049000,
0x08049001, 0x08049008, 0x08049009, 0x08049040, 0x08049041, 0x08049048,
0x08049049, 0x08049200, 0x08049201, 0x08049208, 0x08049209, 0x08049240,
0x08049241, 0x08049248, 0x08049249, 0x08200000, 0x08200001, 0x08200008,
0x08200009, 0x08200040, 0x08200041, 0x08200048, 0x08200049, 0x08200200,
0x08200201, 0x08200208, 0x08200209, 0x08200240, 0x08200241, 0x08200248,
0x08200249, 0x08201000, 0x08201001, 0x08201008, 0x08201009, 0x08201040,
0x08201041, 0x08201048, 0x08201049, 0x08201200, 0x08201201, 0x08201208,
0x08201209, 0x08201240, 0x08201241, 0x08201248, 0x08201249, 0x08208000,
0x08208001, 0x08208008, 0x08208009, 0x08208040, 0x08208041, 0x08208048,
0x08208049, 0x08208200, 0x08208201, 0x08208208, 0x08208209, 0x08208240,
0x08208241, 0x08208248, 0x08208249, 0x08209000, 0x08209001, 0x08209008,
0x08209009, 0x08209040, 0x08209041, 0x08209048, 0x08209049, 0x08209200,
0x08209201, 0x08209208, 0x08209209, 0x08209240, 0x08209241, 0x08209248,
0x08209249, 0x08240000, 0x08240001, 0x08240008, 0x08240009, 0x08240040,
0x08240041, 0x08240048, 0x08240049, 0x08240200, 0x08240201, 0x08240208,
0x08240209, 0x08240240, 0x08240241, 0x08240248, 0x08240249, 0x08241000,
0x08241001, 0x08241008, 0x08241009, 0x08241040, 0x08241041, 0x08241048,
0x08241049, 0x08241200, 0x08241201, 0x08241208, 0x08241209, 0x08241240,
0x08241241, 0x08241248, 0x08241249, 0x08248000, 0x08248001, 0x08248008,
0x08248009, 0x08248040, 0x08248041, 0x08248048, 0x08248049, 0x08248200,
0x08248201, 0x08248208, 0x08248209, 0x08248240, 0x08248241, 0x08248248,
0x08248249, 0x08249000, 0x08249001, 0x08249008, 0x08249009, 0x08249040,
0x08249041, 0x08249048, 0x08249049, 0x08249200, 0x08249201, 0x08249208,
0x08249209, 0x08249240, 0x08249241, 0x08249248, 0x08249249, 0x09000000,
0x09000001, 0x09000008, 0x09000009, 0x09000040, 0x09000041, 0x09000048,
0x09000049, 0x09000200, 0x09000201, 0x09000208, 0x09000209, 0x09000240,
0x09000241, 0x09000248, 0x09000249, 0x09001000, 0x09001001, 0x09001008,
0x09001009, 0x09001040, 0x09001041, 0x09001048, 0x09001049, 0x09001200,
0x09001201, 0x09001208, 0x09001209, 0x09001240, 0x09001241, 0x09001248,
0x09001249, 0x09008000, 0x09008001, 0x09008008, 0x09008009, 0x09008040,
0x09008041, 0x09008048, 0x09008049, 0x09008200, 0x09008201, 0x09008208,
0x09008209, 0x09008240, 0x09008241, 0x09008248, 0x09008249, 0x09009000,
0x09009001, 0x09009008, 0x09009009, 0x09009040, 0x09009041, 0x09009048,
0x09009049, 0x09009200, 0x09009201, 0x09009208, 0x09009209, 0x09009240,
0x09009241, 0x09009248, 0x09009249, 0x09040000, 0x09040001, 0x09040008,
0x09040009, 0x09040040, 0x09040041, 0x09040048, 0x09040049, 0x09040200,
0x09040201, 0x09040208, 0x09040209, 0x09040240, 0x09040241, 0x09040248,
0x09040249, 0x09041000, 0x09041001, 0x09041008, 0x09041009, 0x09041040,
0x09041041, 0x09041048, 0x09041049, 0x09041200, 0x09041201, 0x09041208,
0x09041209, 0x09041240, 0x09041241, 0x09041248, 0x09041249, 0x09048000,
0x09048001, 0x09048008, 0x09048009, 0x09048040, 0x09048041, 0x09048048,
0x09048049, 0x09048200, 0x09048201, 0x09048208, 0x09048209, 0x09048240,
0x09048241, 0x09048248, 0x09048249, 0x09049000, 0x09049001, 0x09049008,
0x09049009, 0x09049040, 0x09049041, 0x09049048, 0x09049049, 0x09049200,
0x09049201, 0x09049208, 0x09049209, 0x09049240, 0x09049241, 0x09049248,
0x09049249, 0x09200000, 0x09200001, 0x09200008, 0x09200009, 0x09200040,
0x09200041, 0x09200048, 0x09200049, 0x09200200, 0x09200201, 0x09200208,
0x09200209, 0x09200240, 0x09200241, 0x09200248, 0x09200249, 0x09201000,
0x09201001, 0x09201008, 0x09201009, 0x09201040, 0x09201041, 0x09201048,
0x09201049, 0x09201200, 0x09201201, 0x09201208, 0x09201209, 0x09201240,
0x09201241, 0x09201248, 0x09201249, 0x09208000, 0x09208001, 0x09208008,
0x09208009, 0x09208040, 0x09208041, 0x09208048, 0x09208049, 0x09208200,
0x09208201, 0x09208208, 0x09208209, 0x09208240, 0x09208241, 0x09208248,
0x09208249, 0x09209000, 0x09209001, 0x09209008, 0x09209009, 0x09209040,
0x09209041, 0x09209048, 0x09209049, 0x09209200, 0x09209201, 0x09209208,
0x09209209, 0x09209240, 0x09209241, 0x09209248, 0x09209249, 0x09240000,
0x09240001, 0x09240008, 0x09240009, 0x09240040, 0x09240041, 0x09240048,
0x09240049, 0x09240200, 0x09240201, 0x09240208, 0x09240209, 0x09240240,
0x09240241, 0x09240248, 0x09240249, 0x09241000, 0x09241001, 0x09241008,
0x09241009, 0x09241040, 0x09241041, 0x09241048, 0x09241049, 0x09241200,
0x09241201, 0x09241208, 0x09241209, 0x09241240, 0x09241241, 0x09241248,
0x09241249, 0x09248000, 0x09248001, 0x09248008, 0x09248009, 0x09248040,
0x09248041, 0x09248048, 0x09248049, 0x09248200, 0x09248201, 0x09248208,
0x09248209, 0x09248240, 0x09248241, 0x09248248, 0x09248249, 0x09249000,
0x09249001, 0x09249008, 0x09249009, 0x09249040, 0x09249041, 0x09249048,
0x09249049, 0x09249200, 0x09249201, 0x09249208, 0x09249209, 0x09249240,
0x09249241, 0x09249248, 0x09249249))
