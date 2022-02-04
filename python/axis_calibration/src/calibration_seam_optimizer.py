##################################################################
# Axis calibration for AW3 using tensorflow
#
# FRAMES:
#     S - Primary cam
#     C - Carriage flange
#     B - Carriage positioner base
#     A - Positioner at theta of scan
#
# Note: Axis calibration is applied at C_to_B (Not B_to_A at theta=0)
# Note:


import os
import glob
import json
import argparse

import numpy as np
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import open3d as o3d
import tensorflow as tf
from scipy.spatial.transform import Rotation as R
from scipy.spatial.ckdtree import cKDTree

from tools import *
from calibration_optimizer import optimize_calibration


def main(base_dir, num_epochs, viz):
    #############################################
    # setup
    #############################################
    DEVICE = "/gpu:0"
    num_epochs = [num_epochs]
    vars_to_train = [[1] * 6]  # ,[0, 1, 0, 0, 0, 0]]  # [x,y,z,r,p,y]
    crop_radius = 0.05

    #############################################
    # load data and transforms
    #############################################
    # load in transforms for all scans i.e. base to tool0_theta for each scan
    transforms = read_transforms(base_dir)

    section_cloud_files = glob.glob(os.path.join(base_dir, "section_cloud_*.ply"))
    section_cloud_files = sorted(section_cloud_files, key=lambda f: int(f.split("/")[-1].split("_")[-1][:-4]))
    print("\n".join(section_cloud_files))

    # load seam
    seam = o3d.io.read_point_cloud(os.path.join(base_dir, "tf_feature_normals_0.ply"))
    seam = seam.rotate(R.from_rotvec([0,0,-np.pi]).as_matrix(), center=[0.,0.,0.])
    seam = seam.transform(invert_tf(transforms["0"]["positioner_base_to_tool0"]))
    seam = seam.transform(invert_tf(transforms["0"]["carriage_flange_to_positioner_base"]))

    # load scan data
    pts_data = []
    pcds = []
    for cloud_file, tf_file in section_cloud_files:
        c = np.random.rand(3)
        c /= np.linalg.norm(c)
        pcd = o3d.io.read_point_cloud(os.path.join(cloud_file)).paint_uniform_color(c)
        pcd = pcd.voxel_down_sample(0.01)
        S_to_C = transforms["SL_to_carriage_flange"]
        pcd = pcd.transform(S_to_C)

        # crop pcd around seam
        pcd_points = np.array(pcd.points)
        pcd_tree = cKDTree(pcd_points)
        cropped_pcd_pts = set()
        for seam_pt in seam_pts:
            inds = pcd_tree.query_ball_point(seam_pt, crop_radius)
            for ind in inds:
                if tuple(cropped_pcd_pts[ind]) not in cropped_pcd_pts:
                    print("adding point")
                    cropped_pcd_pts.add(tuple(cropped_pcd_pts[ind]))
        cropped_pcd_pts = np.array(list(cropped_pcd_pts))
        print(cropped_pcd_pts.shape)
        cropped_pcd = o3d.geometry.PointCloud()
        cropped_pcd.points = o3d.utility.Vector3dVector(cropped_pcd_pts)
        o3d.visualization.draw_geometries([cropped_pcd])
        exit(1)
        pcds.append(cropped_pcd)
        pts_data.append(np.array(cropped_pcd.points))
    del pcd
    print("Loaded {} point clouds".format(len(pts_data)))
    if viz:
        o3d.visualization.draw_geometries(pcds)

    # optimize_calibration(pts_data, transforms, num_epochs, vars_to_train, DEVICE, viz)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--data-dir",
        type=str,
        required=False,
        help="path to dataset",
        # default="/home/madhavun/data/Valmont/valmontCell/8_258_2_1636237067045/",
        default="/home/madhavun/data/Valmont/valmontCell/z_offsets/4_24_2_1636703828840",
        # default = "/home/madhavun/data/Valmont/valmontCell/z_offsets/4_27_2_1636708143345",
        dest="base_dir",
    )
    parser.add_argument(
        "-e",
        "--num-epochs",
        type=int,
        required=False,
        help="number of epochs",
        default=100,
        dest="num_epochs",
    )
    parser.add_argument(
        "-v",
        "--visualize",
        type=int,
        required=False,
        help="flag to show results",
        default=1,
        dest="viz",
    )
    args = parser.parse_args()
    main(args.base_dir, args.num_epochs, args.viz)
