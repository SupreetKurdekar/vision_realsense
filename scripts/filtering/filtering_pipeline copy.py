import opd_utils as utils

import numpy as np
import open3d as o3d
import os
import sys
import matplotlib.pyplot as plt

folder_path = "/home/supreet/realsense/pointCloudCapture/data/scene/overhead_with_martkers_c.ply"
storage_path = "/home/supreet/realsense/pointCloudCapture/data/scene"


pcd = o3d.io.read_point_cloud(folder_path)
plane_model, inliers = pcd.segment_plane(distance_threshold=0.006,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([outlier_cloud])
o3d.io.write_point_cloud(os.path.join(storage_path,"scene_plane_removed.pcd"), outlier_cloud, write_ascii=True, compressed=False, print_progress=False)

