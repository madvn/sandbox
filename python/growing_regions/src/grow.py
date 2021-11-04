import os
import time

import queue

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


# def geodesic_distance()

def grow_region(data_dir, num_neighbors=10, max_distance=0.02):
    model = o3d.io.read_point_cloud(os.path.join(data_dir, "tf_model_cloud_.ply"))
    model.paint_uniform_color([0.6,0.6,0.6])
    model = model.voxel_down_sample(0.002)

    seam = o3d.io.read_point_cloud(os.path.join(data_dir, "tf_feature_normals_0.ply"))
    seam.paint_uniform_color([1,0,0])

    o3d.visualization.draw_geometries([model, seam])

    # prep
    model_pts = np.array(model.points)
    model_normals = np.array(model.normals)
    model_tree = cKDTree(model_pts)
    seam_normals = np.array(seam.normals)
    seam_tree = cKDTree(np.array(seam.points))
    checked_flags = np.zeros(len(model_pts))

    # initialize queue with seam points
    q = queue.Queue()
    for pt in np.array(seam.points):
        q.put((pt,-1))

    # growing region_points here
    region_points = []
    region_normals = []
    while not q.empty():
        this_pt, this_ind = q.get()
        # print(q.qsize())

        # get k nearest neighbors
        _, inds = model_tree.query(this_pt, num_neighbors)

        # check if neighbors are within threshold distance and add to queue
        for ind in inds:
            seam_dist, nearest_seam_ind = seam_tree.query(model_pts[ind], 1)
            # print(seam_dist, _)
            if seam_dist < max_distance:
                # found a valid point in region_points
                # if this point has not been checked before
                # if this_ind == -1 or checked_flags[this_ind] == 0:
                # if this_ind == -1 or checked_flags[this_ind] == 0:
                if this_ind == -1 or checked_flags[ind] == 0:
                    if np.dot(model_normals[ind], seam_normals[nearest_seam_ind]) >= 0.49:
                        q.put((model_pts[ind], ind))
                        region_points.append(model_pts[ind])
                        region_normals.append(model_normals[ind])
                        checked_flags[ind] = 1

        q_proportion = int(q.qsize()*100/(len(model_pts) + len(np.array(seam.points))))
        print("|"*q_proportion+" "*(100-q_proportion), end="\r", flush=True)

    print(f"\nRegion size: {len(region_points)}")

    region_pcd = o3d.geometry.PointCloud()
    region_pcd.points = o3d.utility.Vector3dVector(region_points)
    region_pcd.normals = o3d.utility.Vector3dVector(region_normals)
    region_pcd.paint_uniform_color([0.,0.,1])
    region_pcd  = region_pcd.voxel_down_sample(0.01)

    o3d.visualization.draw_geometries([model, region_pcd])
    o3d.visualization.draw_geometries([region_pcd])
    o3d.io.write_point_cloud("seam_geometry.ply", region_pcd)


grow_region("../test_data/oneBarDoor")
