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
parser.add_argument('-s', '--seed', dest='seed', type=int, default=np.random.randint(10000))


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
    def __init__(self, x, y, z, yaw, r):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.r = r


def gen_node_heights(height: float, node_count: int):
    sample_count = 8
    samples = np.array([np.concatenate([np.sort(np.random.uniform(
        0.005, height, node_count)), [height]]) for _ in range(sample_count)])
    max_distances = np.max(samples[:, 1:] - samples[:, :-1], axis=1)
    return samples[np.argmin(max_distances)]


def gen_main_stem(collection):
    verts = []
    faces = []

    DIV = 23
    height = 1.25 + np.random.normal(0, 0.1)
    r_start = .009 + np.random.normal(0, 0.0001)
    r_end = r_start * 0.42
    node_count = 19 + np.random.randint(6)

    # create the nodes from bottom-up
    nodes = [Node(0, 0, 0, 0, r_start)]
    node_heights = gen_node_heights(height, node_count)
    for z in node_heights:
        prev = nodes[-1]
        yaw = prev.yaw + (np.pi * (2.0/3.0) + np.random.normal(0, 0.4))
        step_up = z - prev.z
        y = np.sin(yaw) * (step_up/15.0)
        x = np.cos(yaw) * (step_up/15.0)
        r = r_start - (r_start-r_end) * (z / height)
        nodes.append(Node(x, y, z, yaw, r))

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

def filter_prefix(objects, prefix):
    return list(
        filter(lambda ob: ob.name.startswith(prefix), objects))

class PrebuiltMeshes:
    def __init__(self):
        objects = bpy.data.collections['sub_stems'].all_objects
        self.sub_stems = filter_prefix(objects, 'b_')

    def get_end_stem_mesh(self, growth):
        groups = ['b_04', 'b_05', 'b_06', 'b_08' ]
        prefix = groups[np.random.choice(len(groups))]
        stems = filter_prefix(self.sub_stems, prefix)
        growths = np.array(list(map(lambda ob: ob.location[2], stems)))
        growths = growths / np.max(growths)
        idx_options = np.argsort(np.abs(growths - growth))[:5]
        return stems[np.random.choice(idx_options)]
prebuilt_meshes = PrebuiltMeshes()

def gen_end_stem(collection, location: (float, float, float), yaw: float, growth: float):
    coll_sub_stems = bpy.data.collections['sub_stems']
    ob_stem = prebuilt_meshes.get_end_stem_mesh(growth)
    ob_fruits = filter_prefix(ob_stem.children, 'pose_fruit')
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
        ob_stem.rotation_euler[2] += yaw

    collection.objects.link(ob_stem)
    coll_sub_stems.objects.unlink(ob_stem)
    for ob in ob_fruits:
        collection.objects.link(ob)
        coll_sub_stems.objects.unlink(ob)

    return ob_stem, ob_fruits




def gen_plant(write_results=False, keep_materials=False):
    collection = bpy.data.collections.new('new_plant')
    bpy.context.scene.collection.children.link(collection)

    ob_main_stem, nodes = gen_main_stem(collection)

    ob_meshes = [ob_main_stem]
    ob_fruits = []

    for node in nodes[1:]:
        ob_m, ob_f = gen_end_stem(
            collection, 
            location = (node.x, node.y, node.z), 
            yaw = node.yaw,
            growth = node.z / nodes[-1].z)
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
        if not keep_materials:
            ob.data.materials.clear()
        # select for the export
        ob.select_set(True)

    if write_results:
        bpy.ops.wm.collada_export(filepath=str(
            model_dir / 'meshes/tomato.dae'), check_existing=True, selected=True)

        with open(model_dir / 'markers.json', 'w') as outfile:
            json.dump(markers, outfile, indent=4)
        bpy.ops.object.delete(use_global=True)

    return collection

if __name__ == '__main__':
    gen_plant(write_results=True)

# if __name__ == '__main__':
#     if 'test_copy' in bpy.data.collections:
#         collection = bpy.data.collections['test_copy']
#     else:
#         collection = bpy.data.collections.new('test_copy')
#         bpy.context.scene.collection.children.link(collection)

#     print(gen_end_stem(collection, (0, 0, 0), 1.0))
