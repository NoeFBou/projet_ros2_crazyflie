import open3d as o3d
import numpy as np
import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description="Convertir un nuage de points (PCD) en maillage 3D (STL).")

    parser.add_argument("-i", "--input", type=str, default="test.pcd",
                        help="Chemin du fichier d'entrée (défaut: test.pcd)")

    parser.add_argument("-o", "--output", type=str, default="output_mesh.stl",
                        help="Chemin du fichier de sortie (défaut: output_mesh.stl)")

    parser.add_argument('--floor', action='store_true', help='Ajouter un sol synthétique artificiel')
    args = parser.parse_args()

    INPUT_FILE = args.input
    OUTPUT_FILE = args.output

    if not os.path.exists(INPUT_FILE):
        print(f"Erreur : Le fichier '{INPUT_FILE}' est introuvable.")
        sys.exit(1)

    print(f" Chargement de {INPUT_FILE}...")
    pcd = o3d.io.read_point_cloud(INPUT_FILE)

    if len(pcd.points) == 0:
        print("Erreur: Le fichier est vide ou mal lu.")
        sys.exit(1)



    print(f"Points initiaux : {len(pcd.points)}")
    if args.floor:
        #print("--- OPTION ACTIVÉE : Génération du sol synthétique ---")

        points = np.asarray(pcd.points)
        min_x, min_y, min_z = points.min(axis=0)
        max_x, max_y, max_z = points.max(axis=0)

        floor_z = min(min_z, 0.0)
        step = 0.05
        x_range = np.arange(min_x, max_x, step)
        y_range = np.arange(min_y, max_y, step)
        xx, yy = np.meshgrid(x_range, y_range)

        floor_points_np = np.zeros((xx.size, 3))
        floor_points_np[:, 0] = xx.flatten()
        floor_points_np[:, 1] = yy.flatten()
        floor_points_np[:, 2] = floor_z

        floor_pcd = o3d.geometry.PointCloud()
        floor_pcd.points = o3d.utility.Vector3dVector(floor_points_np)

        floor_normals = np.zeros((xx.size, 3))
        floor_normals[:, 2] = 1.0
        floor_pcd.normals = o3d.utility.Vector3dVector(floor_normals)

        pcd = pcd + floor_pcd
        print(f"Sol ajoute ({len(floor_points_np)} points).")


    if args.floor:
        pcd = pcd + floor_pcd


    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)
    pcd_clean = pcd.select_by_index(ind)
    #print(f"Points restants : {len(pcd_clean.points)}")

    pcd_clean.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd_clean.orient_normals_consistent_tangent_plane(100)


    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_clean, depth=9)

    vertices_to_remove = densities < np.quantile(densities, 0.1)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    mesh = mesh.filter_smooth_taubin(number_of_iterations=50)

    mesh.compute_vertex_normals()


    print(f" Sauvegarde dans {OUTPUT_FILE}...")
    o3d.io.write_triangle_mesh(OUTPUT_FILE, mesh)


if __name__ == "__main__":
    main()