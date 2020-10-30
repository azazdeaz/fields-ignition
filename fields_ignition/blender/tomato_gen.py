import sys
import bpy
import numpy as np
import re
import json
from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-o', '--model_dir', dest='model_dir', required=True)
parser.add_argument('-s', '--seed', dest='seed', type=int, default=81225)


if '--' in sys.argv:
    argv = sys.argv[sys.argv.index('--') + 1:]
else:
    argv = []
args = parser.parse_known_args(argv)[0]
print(args)
# raise Exception('stop')

np.random.seed(args.seed)

bpy.ops.object.select_all(action='DESELECT')

model_dir = Path(args.model_dir)
model_dir.mkdir(parents=True, exist_ok=True)

model_no = np.random.randint(1, 11)
model_name = 'AG15_{}'.format(model_no)
print('selected model is', model_name)


def select_objects(objects):
    ob_merged = None
    ob_fruits = []

    for ob in objects:
        if ob.name.startswith(model_name):
            ob_merged = ob
        elif ob.parent is not None and ob.parent.name.startswith(model_name):
            if ob.name.startswith('pose_fruit'):
                ob_fruits.append(ob)

    return ob_merged, ob_fruits


ob_merged, ob_fruits = select_objects(bpy.data.objects)
bpy.ops.object.duplicate(
    {'selected_objects': [ob_merged] + ob_fruits, 'active_object': ob_merged})
print('bpy.context.selected_objects', bpy.context.selected_objects)
ob_merged, ob_fruits = select_objects(bpy.context.selected_objects)
ob_merged.location = (0, 0, 0)
ob_merged.rotation_euler[2] = np.pi * 2 * np.random.random()
print(list(ob_fruits))

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

bpy.ops.mesh.separate({'selected_objects': [ob_merged]}, type='MATERIAL')

for ob in bpy.context.selected_objects:
    mat_name = ob.material_slots[0].material.name
    new_name = re.search("AG15_(\w*)", mat_name)[1]
    ob.name = new_name

bpy.ops.wm.collada_export(filepath=str(
    model_dir / 'meshes/tomato.dae'), check_existing=True, selected=True)

with open(model_dir / 'markers.json', 'w') as outfile:
    json.dump(markers, outfile, indent=4)
bpy.ops.object.delete(use_global=True)
