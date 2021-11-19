import sys
import os
import copy
import glob
from multiprocessing import Pool
import argparse

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
import open3d as o3d


def get_point_normal(args):
    point, triangles, verts, normals = args
    min_S = np.inf
    argmin_S = None
    for i, triangle in enumerate(triangles):
        v1, v2, v3 = np.array([verts[i] for i in triangle])
        S = 0.5 * np.linalg.norm(np.cross(v1 - v2, v1 - v3))
        S1 = 0.5 * np.linalg.norm(np.cross(point - v1, point - v2))
        S2 = 0.5 * np.linalg.norm(np.cross(point - v2, point - v3))
        S3 = 0.5 * np.linalg.norm(np.cross(point - v1, point - v3))
        score = abs(S - (S1 + S2 + S3))
        if min_S > score:
            min_S = score
            argmin_S = i
        if score < 1e-10:
            return normals[i]
    return normals[argmin_S]


def get_pcd_normals(m, points):
    """
    function to find better normals for pcd from mesh
    """
    triangles = np.array(m.triangles)
    verts = np.array(m.vertices)
    normals = np.array(m.triangle_normals)

    args = [(point, triangles, verts, normals) for point in points]
    pool = Pool(30)
    pcd_normals = pool.map(get_point_normal, args)

    return pcd_normals


def get_seam_normals(pcd, seam, radius=0.01):
    pcd_kdtree = cKDTree(np.array(pcd.points))
    pcd_normals = np.array(pcd.normals)
    seam_normals = []

    for pt in np.array(seam.points):
        inds = pcd_kdtree.query_ball_point(pt, radius)
        seam_normals.append(np.mean(pcd_normals[inds], axis=0))

    return seam_normals


def make_flat_surface(show=False):
    verts = [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.1, 0.1, 0.0], [0.0, 0.1, 0.0]]
    tris = [[0, 1, 2], [0, 2, 3]]
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(tris)
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()

    if show:
        f = o3d.geometry.TriangleMesh.create_coordinate_frame()
        f = f.scale(0.05, center=[0.0, 0.0, 0.0])
        o3d.visualization.draw_geometries([mesh, f])

    return mesh


###############
# tee joint
###############
def make_line_tee_joint(num_points):
    s1 = make_flat_surface()
    s2 = make_flat_surface()
    s2 = s2.rotate(Rot.from_rotvec([np.pi / 2, 0.0, 0.0]).as_matrix(), center=[0.0, 0.0, 0.0])
    s2 = s2.rotate(Rot.from_rotvec([0.0, 0.0, np.pi]).as_matrix(), center=[0.05, 0.0, 0.0])
    mesh = s1 + s2

    pcd = mesh.sample_points_poisson_disk(num_points)
    pcd = pcd.voxel_down_sample(0.0005)
    normals = get_pcd_normals(mesh, np.array(pcd.points))
    pcd.normals = o3d.utility.Vector3dVector(normals)

    seam_pts = np.linspace(0.0, 0.1, int(1e3)).reshape([-1, 1])
    seam_pts = np.hstack([seam_pts, np.zeros_like(seam_pts), np.zeros_like(seam_pts)])
    seam = o3d.geometry.PointCloud()
    seam.points = o3d.utility.Vector3dVector(seam_pts)
    seam.paint_uniform_color([0.0, 0.0, 0.0])
    normals = get_seam_normals(pcd, seam)
    seam.normals = o3d.utility.Vector3dVector(normals)
    seam.normalize_normals()

    return mesh, pcd, seam


def make_thin_wall_tee_joint(num_points):
    m1, pcd1, seam = make_line_tee_joint(num_points)
    m2 = copy.deepcopy(m1)
    pcd2 = copy.deepcopy(pcd1)
    m2 = m2.rotate(Rot.from_rotvec([np.pi / 2, 0.0, 0.0]).as_matrix(), center=[0.0, 0.0, 0.0])
    pcd2 = pcd2.rotate(Rot.from_rotvec([np.pi / 2, 0.0, 0.0]).as_matrix(), center=[0.0, 0.0, 0.0])
    m2 = m2.translate([0.0, -0.005, 0.0])
    pcd2 = pcd2.translate([0.0, -0.005, 0.0])

    mesh = m1 + m2
    pcd = pcd1 + pcd2

    pcd.paint_uniform_color([0.7, 0.7, 0.7])

    return mesh, pcd, seam


def make_thin_wall_crossbar_joint(num_points):
    s1 = make_flat_surface()
    s2 = make_flat_surface()
    s2 = s2.rotate(Rot.from_rotvec([np.pi / 2, 0.0, 0.0]).as_matrix(), center=[0.0, 0.0, 0.0])
    s2 = s2.rotate(Rot.from_rotvec([0.0, 0.0, np.pi]).as_matrix(), center=[0.05, 0.0, 0.0])
    s1 = s1.scale(2, center=s1.get_center())
    mesh = s1 + s2

    pcd = mesh.sample_points_poisson_disk(num_points)
    pcd = pcd.voxel_down_sample(0.0005)
    normals = get_pcd_normals(mesh, np.array(pcd.points))
    pcd.normals = o3d.utility.Vector3dVector(normals)

    seam_pts = np.linspace(0.0, 0.1, int(1e3)).reshape([-1, 1])
    seam_pts = np.hstack([seam_pts, np.zeros_like(seam_pts), np.zeros_like(seam_pts)])
    seam = o3d.geometry.PointCloud()
    seam.points = o3d.utility.Vector3dVector(seam_pts)
    seam.paint_uniform_color([0.0, 0.0, 0.0])
    normals = get_seam_normals(pcd, seam)
    seam.normals = o3d.utility.Vector3dVector(normals)
    seam.normalize_normals()

    pcd.paint_uniform_color([0.7, 0.7, 0.7])

    return mesh, pcd, seam


if __name__ == "__main__":
    # m,p,s = make_line_tee_joint(1000)
    # m,p,s = make_thin_wall_tee_joint(5000)
    m, p, s = make_thin_wall_crossbar_joint(5000)
    o3d.visualization.draw_geometries([m, p, s])
