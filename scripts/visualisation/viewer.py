import opd_utils as utils

import numpy as np
import open3d as o3d
import os
import sys
import matplotlib.pyplot as plt

folder_path = "/home/supreet/vision2/data/battery_highprec/raw_pcd"
# folder_path = "/home/supreet/thVision/data_flipped/drill_2/filtered"
pcd_list,dirs = utils.load_pc_to_limit_with_downsampling(folder_path,1,0.001)

# # visualiser set up
# vis = o3d.visualization.Visualizer()
# vis.create_window()

# viewer_callback(vis,current,pcd_list)

Trs = np.eye(4)
# Trs[0][0] = -1
# Trs[1][1] = -1

# tr2 = np.eye(4)
# tr2[0][0] = -1
# tr2[2][2] = -1

utils.vis_dict(pcd_list,Trans=Trs)