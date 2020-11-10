import sys
import bpy
import numpy as np
import re
import json
from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-o', '--model_dir', dest='model_dir',
                    default='generated/test_tomato')
parser.add_argument('-s', '--seed', dest='seed', type=int, default=81225)


if '--' in sys.argv:
    argv = sys.argv[sys.argv.index('--') + 1:]
else:
    argv = []
args = parser.parse_known_args(argv)[0]

np.random.seed(args.seed)

bpy.ops.object.select_all(action='DESELECT')

model_dir = Path(args.model_dir)
model_dir.mkdir(parents=True, exist_ok=True)




# Generate the main structure

class Node:
    def __init__(self, x, y, z, alpha, r):
        self.x = x
        self.y = y
        self.z = z
        self.alpha = alpha
        self.r = r


def gen_main_stem(collection):
    verts = []
    faces = []

    DIV = 23
    height = 1.25 + np.random.normal(0, 0.1)
    r_start = .009 + np.random.normal(0, 0.0001)
    r_end = r_start * 0.42
    node_count = 15 + np.random.randint(10)

    nodes = [Node(0, 0, 0, 0, r_start)]
    node_heights = list(np.sort(np.random.uniform(
        0.005, height, node_count))) + [height]
    for z in node_heights:
        prev = nodes[-1]
        alpha = prev.alpha + (np.pi * (2.0/3.0) + np.random.normal(0, 0.1))
        step_up = z - prev.z
        y = np.sin(alpha) * (step_up/15.0)
        x = np.cos(alpha) * (step_up/15.0)
        r = r_start - (r_start-r_end) * (z / height)
        nodes.append(Node(x, y, z, alpha, r))

    def create_ring_verts(node):
        verts = []
        for i in range(DIV):
            a = (np.pi*2) * (i/DIV)
            x = node.x + np.cos(a) * node.r
            y = node.y + np.sin(a) * node.r
            verts.append((x, y, node.z))
        return verts

    prev_ring = create_ring_verts(nodes[0])
    verts = verts + prev_ring
    for node in nodes[1:]:
        #    r = r_start - (r_start - r_end) * (z_step / z_steps)
        vert_start = len(verts) - DIV
        ring = create_ring_verts(node)
        verts = verts + ring
        prev_ring = ring

        for i in range(DIV):
            vi = vert_start + i
            vip = vert_start + ((i+1) % DIV)
            faces.append((vi, vip, vip+DIV, vi+DIV))

    #close main stem
    last_ring_start = len(verts) - DIV
    verts.append((nodes[-1].x, nodes[-1].y, nodes[-1].z + 0.01))
    for i in range(DIV):
        vi = last_ring_start + i
        vip = last_ring_start + ((i+1) % DIV)
        faces.append((vi, vip, len(verts)-1))

    #Define mesh and object
    mesh = bpy.data.meshes.new("Branch1")

    #Create mesh
    mesh.from_pydata(verts, [], faces)
    mesh.update(calc_edges=True)

    object = bpy.data.objects.new("Branch1", mesh)

    collection.objects.link(object)

    mat = bpy.data.materials.get("Branch1")
    object.data.materials.append(mat)

    return object, nodes



# Generate leaf, fruit, and flower formations

# def select_objects(object_name, ob_pool):
#     ob_merged = None
#     ob_fruits = []

#     def is_match(ob):
#         return ob.name == object_name or ob.name.startswith(object_name + '.')

#     for ob in ob_pool:
#         if is_match(ob):
#             ob_merged = ob
#         elif ob.parent is not None and is_match(ob.parent):
#             if ob.name.startswith('pose_fruit'):
#                 ob_fruits.append(ob)
#     if ob_merged is None:
#         raise LookupError(
#             'Can\'t find object with name "{}"'.format(object_name))
#     return ob_merged, ob_fruits

def filter_prefix(objects, prefix):
    return list(
        filter(lambda ob: ob.name.startswith(prefix), objects))

class PrebuiltMeshes:
    def __init__(self):
        objects = bpy.data.collections['sub_branches'].all_objects
        self.sub_branches = filter_prefix(objects, 'b_')
        self.sub_branch_heights = np.array(
            list(map(lambda ob: ob.location[2], self.sub_branches)))

    def get_end_stem_mesh(self, z):
        idx_options = np.argsort(np.abs(self.sub_branch_heights-z))[:5]
        return self.sub_branches[np.random.choice(idx_options)]
prebuilt_meshes = PrebuiltMeshes()

def gen_end_stem(collection, location: (float, float, float), yaw=None):
    coll_sub_branches = bpy.data.collections['sub_branches']
    ob_stem = prebuilt_meshes.get_end_stem_mesh(location[2])
    print('ob_stem', ob_stem, ob_stem.type)
    object_name = ob_stem.name
    # TODO select markers from ob children
    ob_fruits = filter_prefix(ob_stem.children, 'pose_fruit')
    print('ob_fruits', ob_fruits)
    # ob_stem, ob_fruits = select_objects(object_name, sub_branches)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.duplicate({
        'selected_objects': [ob_stem] + ob_fruits,
        'active_object': ob_stem
    })
    ob_stem = filter_prefix(bpy.context.selected_objects, 'b_')[0]
    ob_fruits = filter_prefix(ob_stem.children, 'pose_fruit')
    
    ob_stem.select_set(False)
    if location is not None:
        ob_stem.location = location
    if yaw is not None:
        ob_stem.rotation_euler[2] = yaw

    collection.objects.link(ob_stem)
    coll_sub_branches.objects.unlink(ob_stem)
    for ob in ob_fruits:
        collection.objects.link(ob)
        coll_sub_branches.objects.unlink(ob)

    return ob_stem, ob_fruits




def gen_plant(write_result=False):
    collection = bpy.data.collections.new('new_plant')
    bpy.context.scene.collection.children.link(collection)

    ob_main_stem, nodes = gen_main_stem(collection)

    ob_meshes = [ob_main_stem]
    ob_fruits = []

    for node in nodes:
        mesh_name = 'b_0' if np.random.random_sample() > .5 else 'b_1'
        ob_m, ob_f = gen_end_stem(collection, (node.x, node.y, node.z), node.alpha)
        ob_meshes.append(ob_m)
        ob_fruits += ob_f

    markers = []


    def add_marker(location, type='FRUIT'):
        marker = {
            'marker_type': type,
            'translation': [location[0], location[1], location[2]],
        }
        markers.append(marker)


    for ob in ob_fruits:
        bpy.ops.object.parent_clear(
            {'selected_objects': [ob]}, type='CLEAR_KEEP_TRANSFORM')
        add_marker(ob.location)
        bpy.ops.object.delete({'selected_objects': [ob]}, use_global=True)

    bpy.ops.object.join({
        'selected_objects': ob_meshes,
        'selected_editable_objects': ob_meshes,
        'active_object': ob_meshes[0]
    })

    ob_merged = collection.objects[0]

    bpy.ops.mesh.separate({
        'selected_editable_objects': [ob_merged]
    }, type='MATERIAL')

    bpy.ops.object.select_all(action='DESELECT')
    for ob in collection.all_objects:
        # name the mesh
        ob.name = ob.material_slots[0].material.name
        # remove the material (it will be added in the SDF file)
        ob.data.materials.clear()
        # select for the export
        ob.select_set(True)

    if write_result:
        bpy.ops.wm.collada_export(filepath=str(
            model_dir / 'meshes/tomato.dae'), check_existing=True, selected=True)

        with open(model_dir / 'markers.json', 'w') as outfile:
            json.dump(markers, outfile, indent=4)
        bpy.ops.object.delete(use_global=True)

    return collection

if __name__ == '__main__':
    gen_plant(write_result=True)

# if __name__ == '__main__':
#     if 'test_copy' in bpy.data.collections:
#         collection = bpy.data.collections['test_copy']
#     else:
#         collection = bpy.data.collections.new('test_copy')
#         bpy.context.scene.collection.children.link(collection)

#     print(gen_end_stem(collection, (0, 0, 0), 1.0))
