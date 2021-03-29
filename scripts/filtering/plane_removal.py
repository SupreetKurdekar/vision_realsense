import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import filtering_utils as futils


pcd = o3d.io.read_point_cloud("/home/supreet/vision2/data/right_handle/right_handle_scene_cleaned_1.ply")
storage_path = "/home/supreet/vision2/data/right_handle"
o3d.visualization.draw_geometries([pcd])

_, inliers = pcd.segment_plane(distance_threshold=0.006,
                                        ransac_n=3,
                                        num_iterations=1000)

inlier_cloud = pcd.select_by_index(inliers,invert = True)


cl, ind = inlier_cloud.remove_radius_outlier(nb_points=50, radius=0.005)



futils.display_inlier_outlier(inlier_cloud, ind)

final_cld = inlier_cloud.select_by_index(ind)

o3d.io.write_point_cloud(os.path.join(storage_path,"final_cleaned_right_handle.pcd"), final_cld, write_ascii=True, compressed=False, print_progress=False)
o3d.io.write_point_cloud(os.path.join(storage_path,"final_cleaned_right_handle.ply"), final_cld, write_ascii=True, compressed=False, print_progress=False)