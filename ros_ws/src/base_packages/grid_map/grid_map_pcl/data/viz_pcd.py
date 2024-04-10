import open3d as o3d
import matplotlib.pyplot as plt

# Load PCD file
pcd = o3d.io.read_point_cloud("input_cloud.pcd")

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
