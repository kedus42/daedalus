import numpy as np
import open3d as o3d

input_path="/home/kedus/Workspace/bags/"
output_path="/home/kedus/Workspace/bags/"
dataname="cave_world_map.pcd"
pcd = o3d.io.read_point_cloud(input_path+dataname)
pcd = pcd.voxel_down_sample(voxel_size=0.05)

alpha = 0.03
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)