import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import filtering_utils as futils
import os

print("Radius oulier removal")

pcd = o3d.io.read_point_cloud("/home/supreet/vision2/data/alignment_demo/battery.pcd")
storage_path = "/home/supreet/vision2/data/battery/filtered_pcds"

cl, ind = pcd.remove_radius_outlier(nb_points=16, radius=0.05)

pcd = pcd.select_by_index(ind)

labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

# max_label = labels.max()
max_label = np.amax(labels)
labels[labels== -1] = max_label + 1
print(f"point cloud has {max_label + 1} clusters")
# colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
# colors[labels < 0] = 0
# pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([pcd])

greatest_label = np.bincount(labels).argmax()


final_cld = o3d.geometry.PointCloud()
final_cld.points = o3d.utility.Vector3dVector(np.array(pcd.points)[labels == greatest_label])
# o3d.visualization.draw_geometries([final_cld])

cl, ind = final_cld.remove_radius_outlier(nb_points=50, radius=0.005)



futils.display_inlier_outlier(final_cld, ind)

final_cld = final_cld.select_by_index(ind)

o3d.io.write_point_cloud(os.path.join(storage_path,"final_cleaned_battery.pcd"), final_cld, write_ascii=True, compressed=False, print_progress=False)

