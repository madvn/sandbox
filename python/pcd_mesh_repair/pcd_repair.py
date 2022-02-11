import tqdm
import numpy as np
import open3d as o3d

from scipy.spatial.ckdtree import cKDTree
from scipy.spatial.transform import Rotation as Rot

from sklearn.decomposition import PCA


def pcd_upsample(filename, query_dist=0.01, normal_threshold=0.5):
    pcd = o3d.io.read_point_cloud(filename)
    pcd.paint_uniform_color([0.5, 0.0, 0])
    cloud_pts = np.array(pcd.points)
    cloud_normals = np.array(pcd.normals)

    kdtree = cKDTree(cloud_pts)

    pts = []
    normals = []
    for point, normal in zip(tqdm.tqdm(cloud_pts), cloud_normals):
        neighbors_inds = kdtree.query_ball_point(point, query_dist)
        for ind in neighbors_inds[:10]:
            if np.dot(normal, cloud_normals[ind]) > normal_threshold:
                pts.append((point + cloud_pts[ind]) / 2)
                normals.append((normal + cloud_normals[ind]) / 2)

    filler_pcd = o3d.geometry.PointCloud()
    filler_pcd.points = o3d.utility.Vector3dVector(pts)
    filler_pcd.normals = o3d.utility.Vector3dVector(normals)
    filler_pcd.paint_uniform_color([0, 0.0, 0.5])

    o3d.io.write_point_cloud("./upsampled_pcd.ply", filler_pcd + pcd)
    # o3d.visualization.draw_geometries([filler_pcd, pcd])


def rotate_and_concatenate(filename, num_steps=10, dist_threshold=0.002):
    """
    1. find principal axis
    2. rotate about principal axis and make concat_pcd
    3. subtract against original pcd to only get filler regions

    :param filename:
    :param num_steps:
    :return:
    """
    pcd = o3d.io.read_point_cloud(filename)
    pcd.paint_uniform_color([0.5, 0.0, 0])

    pca = PCA(n_components=3)
    pca.fit(np.array(pcd.points))
    axis_ind = np.argmax([np.abs(np.dot(c, [0, 0, 1])) for c in pca.components_])
    axis = pca.components_[axis_ind]
    axis /= np.linalg.norm(axis)
    print(axis)

    concat_pcd = o3d.geometry.PointCloud()
    for theta in np.linspace(0, np.pi, num_steps):
        # rot_mat = Rot.from_rotvec([0,0,theta]).as_matrix()
        rot_mat = Rot.from_rotvec(axis * theta).as_matrix()
        concat_pcd += pcd.rotate(rot_mat)

    concat_pcd.paint_uniform_color([0.0, 0.5, 0.0])
    concat_pcd.voxel_down_sample(0.002)

    kdtree = cKDTree(np.array(pcd.points))
    diff_points = []
    diff_normals = []
    for point, normal in zip(tqdm.tqdm(np.array(concat_pcd.points), desc="pcd subtract"), np.array(concat_pcd.normals)):
        dist, _ = kdtree.query(point)
        if dist > dist_threshold:
            diff_points.append(point)
            diff_normals.append(normal)

    diff_cloud = o3d.geometry.PointCloud()
    diff_cloud.points = o3d.utility.Vector3dVector(diff_points)
    diff_cloud.normals = o3d.utility.Vector3dVector(diff_normals)
    diff_cloud.paint_uniform_color([0.0, 0.0, 0.5])
    o3d.visualization.draw_geometries([diff_cloud, pcd])
    o3d.io.write_point_cloud("./rotate_concat_pcd.ply", diff_cloud + pcd)


pcd_upsample("/home/madhavun/data/Valmont/valmontCell/83_1080_2_1644337816367/cloud_composite_normals.ply")
rotate_and_concatenate("/home/madhavun/data/Valmont/valmontCell/83_1080_2_1644337816367/cloud_composite_normals.ply")
