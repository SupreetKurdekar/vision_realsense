import numpy as np
import open3d as o3d
import os
import sys
from scipy.spatial.transform import Rotation as R

def load_pc_to_limit_with_downsampling(path,limit,voxel_size=0.0):
    pcds = []
    i = 0
    dirs = sorted(os.listdir(path))
    for file in dirs:
        if i < limit:
            print(file)
            pcd = o3d.io.read_point_cloud(os.path.join(path,file))
            pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
            pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
            pcds.append(pcd_down)
            i += 1
    return pcds,dirs

def load_pc_in_range_with_downsampling(path,limit_f,voxel_size=0.0,limit_i=0):
    pcds = []
    dirs = sorted(os.listdir(path))
    dirs = dirs[limit_i:limit_f+1]
    for file in dirs:
        print(file)
        pcd = o3d.io.read_point_cloud(os.path.join(path,file))
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        pcds.append(pcd_down)
    return pcds

def load_pc_in_range(path,limit_f,limit_i=0):
    pcds = []
    dirs = sorted(os.listdir(path))
    dirs = dirs[limit_i:limit_f+1]
    for file in dirs:
        print(file)
        pcd = o3d.io.read_point_cloud(os.path.join(path,file))
        pcds.append(pcd_down)
    return pcds

def load_pc(path,radius=0.01, max_nn=30):
    pcds = []
    dirs = sorted(os.listdir(path))
    for file in dirs:
        print(file)
        pcd = o3d.io.read_point_cloud(os.path.join(path,file))
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
        pcds.append(pcd)
    return pcds

def load_pc_return_filenames(path,radius=0.01, max_nn=30):
    pcds = []
    dirs = sorted(os.listdir(path))
    for file in dirs:
        print(file)
        pcd = o3d.io.read_point_cloud(os.path.join(path,file))
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
        pcds.append(pcd)
    return pcds,dirs

def load_pc_with_normals(path):
    pcds = []
    dirs = sorted(os.listdir(path))
    for file in dirs:
        print(file)
        pcd = o3d.io.read_point_cloud(os.path.join(path,file))
        pcds.append(pcd)
    return pcds

def vis_dict(pcd_list,Trans = np.eye(4)):
    # 
    ## display all pointclouds in a list of pointclouds
    # sorted in lexicographic order
    # use keys B and N for back and next

    pcd_list = [pcd.transform(Trans) for pcd in pcd_list]

    vis=o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    idx=0
    pcd=pcd_list[idx]

    vis.add_geometry(pcd)
    #vis.update_renderer()
        
    def show_next(vis):
        nonlocal idx
        if (idx < len(pcd_list)-1):
            
            idx=idx+1
            print("showing ", idx)
            vis.clear_geometries()
            pcd=pcd_list[idx]
            vis.add_geometry(pcd)

    def show_prev(vis):
        nonlocal idx
        if (idx > 0):
            idx=idx-1
            print("showing ", idx)
            vis.clear_geometries()
            pcd=pcd_list[idx]
            vis.add_geometry(pcd)
        
    def exit_key(vis):
        vis.destroy_window()
        
    vis.register_key_callback(ord("M"),show_next)
    vis.register_key_callback(ord("B"),show_prev)

    vis.register_key_callback(32,exit_key)
    vis.poll_events()
    vis.run()
    #vis.destroy_window() 


def get_rotation_matrix(theta_deg,axis = "z"):
    r = R.from_euler(axis, theta_deg, degrees=True).as_matrix()
    c = np.array([[0],[0],[0]])
    r = np.append(r, c, axis=1)
    b = np.array([0,0,0,1])
    r = np.append(r, b)
    return r
