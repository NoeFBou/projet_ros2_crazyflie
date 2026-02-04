import open3d as o3d
import numpy as np
import argparse
import os
import sys

def main():
    parser = argparse.ArgumentParser(description="Outil de conversion PCD vers Voxel Map ou STL (Multi-fichiers)")

    parser.add_argument("-i", "--input", type=str, nargs='+', default=["test.pcd"], 
                        help="Liste des fichiers d'entree (ex: -i scan1.pcd scan2.pcd)")
    
    parser.add_argument("-o", "--output", type=str, default="output_map.ply", 
                        help="Fichier de sortie unique (defaut: output_map.ply)")
    
    parser.add_argument('--voxel-size', type=float, default=0.08, 
                        help="Resolution (Taille du voxel). Defaut: 0.08")
    
    parser.add_argument('--floor', action='store_true', 
                        help="Ajouter un sol synthetique dense")
    
    parser.add_argument('--stl', action='store_true', 
                        help="Activer le mode Maillage (STL).")
    
    parser.add_argument('-v', '--visualize', action='store_true', 
                        help="Visualiser le resultat a la fin")

    args = parser.parse_args()

    OUTPUT_FILE = args.output

    combined_pcd = o3d.geometry.PointCloud()
    files_loaded = 0

    print("--- Debut du chargement ---")
    
    for file_path in args.input:
        if not os.path.exists(file_path):
            print(f"Attention : Le fichier '{file_path}' est introuvable. Ignore.")
            continue

        print(f"Lecture de {file_path}...")
        temp_pcd = o3d.io.read_point_cloud(file_path)

        if len(temp_pcd.points) == 0:
            print(f"Attention : '{file_path}' est vide ou mal lu.")
            continue
        
        combined_pcd += temp_pcd
        files_loaded += 1

    if files_loaded == 0 or len(combined_pcd.points) == 0:
        print("Erreur: Aucun nuage de points valide charge.")
        sys.exit(1)

    print(f"Total points fusionnes : {len(combined_pcd.points)}")

    cl, ind = combined_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = combined_pcd.select_by_index(ind)

    if args.floor:
        points = np.asarray(pcd.points)
        if len(points) > 0:
            min_x, min_y, min_z = points.min(axis=0)
            max_x, max_y, max_z = points.max(axis=0)

            floor_z = min(min_z, 0.0)
            
            step = args.voxel_size / 3.0 
            
            print(f"Generation du sol (Z={floor_z:.2f})...")

            x_range = np.arange(min_x - 1.0, max_x + 1.0, step)
            y_range = np.arange(min_y - 1.0, max_y + 1.0, step)
            xx, yy = np.meshgrid(x_range, y_range)

            floor_points_np = np.zeros((xx.size, 3))
            floor_points_np[:, 0] = xx.flatten()
            floor_points_np[:, 1] = yy.flatten()
            floor_points_np[:, 2] = floor_z

            floor_pcd = o3d.geometry.PointCloud()
            floor_pcd.points = o3d.utility.Vector3dVector(floor_points_np)
            
            floor_pcd.paint_uniform_color([0.5, 0.5, 0.5])

            pcd = pcd + floor_pcd
            print(f"Sol ajoute ({len(floor_points_np)} points).")

    final_geometry = None

    if args.stl:  
        if OUTPUT_FILE.endswith(".ply") and "output" in OUTPUT_FILE:
             OUTPUT_FILE = OUTPUT_FILE.replace(".ply", ".stl")

        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel_size * 2, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(100)

        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)

        vertices_to_remove = densities < np.quantile(densities, 0.1)
        mesh.remove_vertices_by_mask(vertices_to_remove)

        mesh = mesh.filter_smooth_taubin(number_of_iterations=10)
        mesh.compute_vertex_normals()

        print(f"Sauvegarde du STL dans {OUTPUT_FILE}...")
        o3d.io.write_triangle_mesh(OUTPUT_FILE, mesh)
        final_geometry = mesh

    else:
        print(f"Voxelisation (Taille={args.voxel_size})...")
        
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=args.voxel_size)
        
        print(f"Sauvegarde de la VoxelGrid dans {OUTPUT_FILE}...")
        o3d.io.write_voxel_grid(OUTPUT_FILE, voxel_grid)
        final_geometry = voxel_grid

    if args.visualize and final_geometry is not None:
        window_name = "Resultat STL" if args.stl else "Resultat Voxel"
        print("Ouverture de la fenetre...")
        o3d.visualization.draw_geometries([final_geometry], 
                                          window_name=window_name,
                                          width=800, height=600)
    
    print("Termine.")

if __name__ == "__main__":
    main()