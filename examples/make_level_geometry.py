from random import random

from panda3d.core import Vec3
from panda3d.core import InternalName
from panda3d.core import NodePath
from panda3d.core import Geom
from panda3d.core import GeomNode
from panda3d.core import GeomVertexArrayFormat
from panda3d.core import GeomVertexFormat
from panda3d.core import GeomVertexData
from panda3d.core import GeomVertexWriter
from panda3d.core import GeomTriangles
from direct.showbase.ShowBase import ShowBase

from geometry_defs import playground, minilevel


def make_geom(vertices, triangles):
    v_array_format = GeomVertexArrayFormat()
    v_array_format.add_column(
        InternalName.get_vertex(),
        3, Geom.NT_float32, Geom.C_point,
    )
    v_array_format.add_column(
        InternalName.get_color(),
        4, Geom.NT_float32, Geom.C_color,
    )
    v_format = GeomVertexFormat()
    v_format.add_array(v_array_format)
    v_format = GeomVertexFormat.register_format(v_format)
    v_data = GeomVertexData("Data", v_format, Geom.UH_static)
    v_data.unclean_set_num_rows(len(vertices))
    vertex = GeomVertexWriter(v_data, InternalName.get_vertex())
    color = GeomVertexWriter(v_data, InternalName.get_color())

    for v in vertices:
        vertex.set_data3f(v)
        color.set_data4f(random(), random(), random(), 1)

    tris = GeomTriangles(Geom.UHStatic)
    for idx in range(int(len(triangles)/3)):
        base = idx * 3
        tris.add_vertices(
            triangles[base],
            triangles[base+1],
            triangles[base+2],
        )

    geom = Geom(v_data)
    geom.add_primitive(tris)
    node = GeomNode('geom_node')
    node.add_geom(geom)
    surface = NodePath(node)
    return surface


def make_square(element_def):
    origin = Vec3(element_def['origin'])
    vector_a = Vec3(element_def['vector_a'])
    vector_b = Vec3(element_def['vector_b'])
    print(f"Square {origin} {vector_a} {vector_b}")
    geom = make_geom(
        [
            origin,
            origin + vector_a,
            origin + vector_b,
            origin + vector_a + vector_b,
        ],
        [
            0, 1, 2,
            2, 1, 0,
            1, 3, 2,
            2, 3, 1,
        ],
    )
    return geom
    

def make_fence(element_def):
    origin = Vec3(element_def['origin'])
    vector_a = Vec3(element_def['vector_a'])
    vector_b = Vec3(element_def['vector_b'])
    vector_c = Vec3(element_def['vector_c'])
    print(f"Fence {origin} {vector_a} {vector_b} {vector_c}")
    geom = make_geom(
        [
            origin,
            origin + vector_c,
            origin + vector_a,
            origin + vector_a + vector_c,
            origin + vector_b,
            origin + vector_b + vector_c,
            origin + vector_a + vector_b,
            origin + vector_a + vector_b + vector_c,
        ],
        [
            0, 1, 2, 2, 1, 0, 1, 3, 2, 2, 3, 1,
            0, 1, 4, 4, 1, 0, 1, 5, 4, 4, 5, 1,
            2, 3, 6, 6, 3, 2, 3, 6, 7, 7, 6, 3,
            4, 5, 6, 6, 5, 4, 5, 6, 7, 7, 6, 5,
        ],
    )
    return geom
    

def make_block(element_def):
    origin = Vec3(element_def['origin'])
    vector_a = Vec3(element_def['vector_a'])
    vector_b = Vec3(element_def['vector_b'])
    vector_c = Vec3(element_def['vector_c'])
    print(f"Block {origin} {vector_a} {vector_b} {vector_c}")
    geom = make_geom(
        [
            origin,
            origin + vector_c,
            origin + vector_a,
            origin + vector_a + vector_c,
            origin + vector_b,
            origin + vector_b + vector_c,
            origin + vector_a + vector_b,
            origin + vector_a + vector_b + vector_c,
        ],
        [
            0, 2, 1, 1, 2, 3,
            4, 0, 1, 4, 1, 5,
            1, 3, 5, 5, 3, 7,
            2, 6, 3, 6, 7, 3,
            4, 5, 7, 7, 6, 4,
            0, 4, 2, 2, 4, 6, 
        ],
    )
    return geom
    

geom_funcs = dict(
    square=make_square,
    fence=make_fence,
    block=make_block,
)


def make_geometry(level_def):
    level = NodePath("level")
    geoms = []
    for element_def in level_def:
        geom_type = element_def['type']
        if geom_type in geom_funcs:
            geoms.append(geom_funcs[geom_type](element_def))
    for geom in geoms:
        geom.reparent_to(level)
    return level


if __name__ == '__main__':
    levels = dict(
        playground=playground,
        minilevel=minilevel,
    )
    ShowBase()
    base.accept('escape', base.task_mgr.stop)
    base.cam.set_pos(-40, -40, 40)
    base.cam.look_at(0, 0, 0)
    for level_name, level_definition in levels.items():
        print(f"Processing {level_name}")
        level = make_geometry(level_definition)
        level.reparent_to(base.render)
        level.write_bam_file(f"{level_name}.bam")
    print("Done!")
