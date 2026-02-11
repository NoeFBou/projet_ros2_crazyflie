import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import tf_transformations

def convert_sdf_to_ply(sdf_file, output_file):
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    scene = trimesh.Scene()

    for model in root.findall(".//model"):
        model_name = model.get('name')
        if model_name in ['ground_plane', 'crazyflie']:
            continue

        pose_text = model.find('pose').text if model.find('pose') is not None else "0 0 0 0 0 0"
        vals = [float(x) for x in pose_text.split()]
        
        pos = vals[:3]
        euler = vals[3:]
        matrix = tf_transformations.euler_matrix(euler[0], euler[1], euler[2])
        matrix[0:3, 3] = pos

        box_elem = model.find(".//box/size")
        if box_elem is not None:
            size = [float(x) for x in box_elem.text.split()]
            mesh = trimesh.creation.box(extents=size)
            mesh.apply_transform(matrix)
            scene.add_geometry(mesh)

    combined = scene.dump(concatenate=True)
    combined.export(output_file)
    print(f"Succès ! Fichier généré : {output_file}")

if __name__ == "__main__":
    sdf_path = "test3.sdf"
    output_path = "map_gazebo3.ply"
    convert_sdf_to_ply(sdf_path, output_path)
