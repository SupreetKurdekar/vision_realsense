import numpy as np
import open3d as o3d
import os
import sys
import random 
 
# def random_color_generator(n): 
#     ret = [] 
#     r = int(random.random() * 256) 
#     g = int(random.random() * 256) 
#     b = int(random.random() * 256) 
#     step = 256 / n 
#     for i in range(n): 
#     r += step 
#     g += step 
#     b += step 
#     r = int(r) % 256 
#     g = int(g) % 256 
#     b = int(b) % 256 
#     ret.append((r,g,b))  
#     return ret 

def dbscan_clustering(pcd):

    labels = np.array(inlier_cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
    max_label = np.amax(labels)
    labels[labels== -1] = max_label + 1

    # colors = random_color_generator(max_label + 2)

    # for label,color in zip(np.arange(max_label+1),colors):

    for id in range(max_label+2):
        pcd_new = o3d.geometry.PointCloud()
        pcd_new.points = labels==id


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


