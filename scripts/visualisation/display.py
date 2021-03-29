import open3d as o3d

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("/home/supreet/realsense/pointCloudCapture/data/scene/scene_plane_removed.pcd")
pcd2 = o3d.io.read_point_cloud("/home/supreet/realsense/pointCloudCapture/data/battery/filtered_scene/registeredScene_voxel_002.ply")


o3d.visualization.draw_geometries([pcd,pcd2])