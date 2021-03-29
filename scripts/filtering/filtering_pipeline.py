import opd_utils as utils

import numpy as np
import open3d as o3d
import os
import sys
import matplotlib.pyplot as plt
folder_path = "/home/supreet/realsense/pointCloudCapture/data/battery/raw_pcd"
storage_path = "/home/supreet/realsense/pointCloudCapture/data/battery/filtered_pcds"
# folder_path = "/home/supreet/thVision/data_flipped/drill_2/filtered"
pcd_list,dirs = utils.load_pc_to_limit_with_downsampling(folder_path,limit = 1,voxel_size=0.001)

cut_off_dist = 3
x_cutoff = 200

filtered_pcds = []

for id,pcd in enumerate(pcd_list):
    points = np.array(pcd.points)
    colors = np.array(pcd.colors)
    # removing x range points 
    id1 = points[:,0] < x_cutoff
    points = points[id1]
    colors = colors[id1]

    dists = np.linalg.norm(points,axis=1)

    filtered = np.squeeze(points[dists<cut_off_dist])
    colors = colors[dists<cut_off_dist]

    pcd_new = o3d.geometry.PointCloud()
    pcd_new.points = o3d.utility.Vector3dVector(filtered)
    pcd_new.colors = o3d.utility.Vector3dVector(colors)

    _, inliers = pcd_new.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)

    inlier_cloud = pcd_new.select_by_index(inliers,invert = True)


    labels = np.array(inlier_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
    max_label = np.amax(labels)
    labels[labels== -1] = max_label + 1

    greatest_label = np.bincount(labels).argmax()
    

    final_cld = o3d.geometry.PointCloud()
    final_cld.points = o3d.utility.Vector3dVector(np.array(inlier_cloud.points)[labels == greatest_label])
    final_cld.colors = o3d.utility.Vector3dVector(np.array(inlier_cloud.colors)[labels == greatest_label])

    filtered_pcds.append(final_cld)
    o3d.io.write_point_cloud(os.path.join(storage_path,dirs[id][:-2]+"cd"), inlier_cloud, write_ascii=True, compressed=False, print_progress=False)

Trs = np.eye(4)
# Trs[0][0] = -1
# Trs[1][1] = -1

# tr2 = np.eye(4)
# tr2[0][0] = -1
# tr2[2][2] = -1

utils.vis_dict(filtered_pcds,Trans=Trs)


