import os
import time

import queue

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
from tools import make_thin_wall_tee_joint, make_thin_wall_crossbar_joint


class PointDistTree:
    def __init__(self, head):
        self.head = head
        self.node = head

    # def add(self, node):
    #     self.


class PointDist:
    def __init__(self, xyz, value, parent):
        self.xyz = xyz
        self.value = value
        self.parent = parent


def grow_region(data_dir, num_neighbors=5, max_distance=0.05, voxel_size=0.002, show_every_iter=True):
    # model = o3d.io.read_point_cloud(os.path.join(data_dir, "tf_model_cloud_.ply"))
    # model.paint_uniform_color([0.6,0.6,0.6])
    #
    # seam = o3d.io.read_point_cloud(os.path.join(data_dir, "tf_feature_normals_0.ply"))
    # seam.paint_uniform_color([1,0,0])

    _, model, seam = make_thin_wall_crossbar_joint(10000)

    # o3d.visualization.draw_geometries([model, seam])

    # prep
    model = model.voxel_down_sample(voxel_size)
    model_pts = np.array(model.points)
    model_normals = np.array(model.normals)
    model_tree = cKDTree(model_pts)
    seam_pts = np.array(seam.points)
    seam_normals = np.array(seam.normals)
    seam_tree = cKDTree(np.array(seam.points))
    checked_flags = np.zeros(len(model_pts))

    point_sets = []
    normal_sets = []
    point_sets.append(np.array(seam.points))

    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.view_init(60, 35)
    plt.ion()
    plt.show()

    while len(point_sets) < max_distance / voxel_size:
        next_level_points = []
        next_level_normals = []
        for pt in point_sets[-1]:
            # get k nearest neighbors
            _, inds = model_tree.query(pt, num_neighbors)

            # check if neighbors are within threshold distance and add to queue:
            for ind in inds:
                seam_dist, nearest_seam_ind = seam_tree.query(model_pts[ind], 1)
                # print(seam_dist, _)
                if (
                    seam_dist < max_distance
                    and np.dot(seam_normals[nearest_seam_ind], model_normals[ind]) >= 0.0
                    and np.dot(seam_normals[nearest_seam_ind], model_pts[ind] - seam_pts[nearest_seam_ind]) >= 0.0
                ):
                    if checked_flags[ind] == 0:
                        next_level_points.append(model_pts[ind])
                        next_level_normals.append(model_normals[ind])
                    checked_flags[ind] = 1
        if len(next_level_points) > 0:
            point_sets.append(next_level_points)
            normal_sets.append(next_level_normals)
            print(len(point_sets))
        else:
            break

        if show_every_iter:
            ax.cla()
            all_pts = np.concatenate(point_sets)
            ax.scatter3D(model_pts[::2, 0], model_pts[::2, 1], model_pts[::2, 2], s=1, color="b", alpha=0.2)
            ax.scatter3D(all_pts[:, 0], all_pts[:, 1], all_pts[:, 2], s=1, color="r")
            ax.set_zlim([0.0, 0.1])
            plt.draw()
            plt.pause(0.01)
            plt.savefig(f"./grow_{len(point_sets)}.png")

            # region_pcd = o3d.geometry.PointCloud()
            # region_pcd.points = o3d.utility.Vector3dVector(all_pts)
            # region_pcd.normals = o3d.utility.Vector3dVector(np.concatenate(normal_sets))
            # region_pcd.paint_uniform_color([0.,0.,1])
            # o3d.visualization.draw_geometries([region_pcd])

    # showing final results
    region_pcd = o3d.geometry.PointCloud()
    region_pcd.points = o3d.utility.Vector3dVector(np.concatenate(point_sets))
    region_pcd.normals = o3d.utility.Vector3dVector(np.concatenate(normal_sets))
    region_pcd.paint_uniform_color([0.0, 0.0, 1])
    # region_pcd  = region_pcd.voxel_down_sample(0.01)

    o3d.visualization.draw_geometries([region_pcd])
    o3d.visualization.draw_geometries([model, region_pcd])
    # o3d.io.write_point_cloud("seam_geometry.ply", region_pcd)


grow_region("../test_data/oneBarDoor")
