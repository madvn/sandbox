import os
import sys
import subprocess
import copy

import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA


class BaseplatePlaneFittingParams:
    def __init__(self, nominal_axis=2, plane_noise=0.001, ransac_min_count=10000, ransac_num_iters=1000, show=False):
        """
        Params for baseplate plane fitting
        :param nominal_axis: provide from [0,1,2] for [x,y,z] nominal pole-axis respectively
        :param plane_noise: noise level expected in the baseplate region of the scan
        :param ransac_min_count: min number of initial points for ransac plane segmentation
        :param ransac_num_iters: number of iters for plane segmentation
        :param show: flag for visualizing results
        """
        self.nominal_axis = nominal_axis
        self.plane_noise = plane_noise
        self.ransac_min_count = ransac_min_count
        self.ransac_num_iters = ransac_num_iters
        self.show = show


def get_circle_pcd(pcd, plane_pcd, axis, offset):
    """
    slice pcd using plane_pcd after applying an offset along the specified axis
    """
    noisy_plane_pcd = copy.deepcopy(plane_pcd).translate(axis * offset)
    tmp_pts = np.array(noisy_plane_pcd.points)
    tmp_pts += np.random.rand(tmp_pts.shape[0], 3) * 1e-4
    noisy_plane_pcd.points = o3d.utility.Vector3dVector(tmp_pts)
    plane_bbox = noisy_plane_pcd.get_oriented_bounding_box()
    # o3d.visualization.draw_geometries([pcd, plane_bbox])
    circle_pcd = pcd.crop(plane_bbox)
    return noisy_plane_pcd, plane_bbox, circle_pcd


def get_max_point_distance(points):
    """
    There's got to be a better way to do this
    open3d's get_point_cloud_distance isn't it
    """
    diameter = 0.0
    for i in range(len(points)):
        for j in range(i, len(points)):
            p1, p2 = points[i], points[j]
            diameter = np.max([diameter, np.linalg.norm(p1 - p2)])
    return diameter


def crop_plane_with_circle(circle_pcd, circle_center, noisy_plane_pcd, plane_pcd, scale=1.0, invert=False):
    """
    crop plane_pcd to remove points inside circle_pcd
    """
    circle_pcd = circle_pcd.translate(-circle_center)
    noisy_plane_pcd = noisy_plane_pcd.translate(-circle_center)
    # o3d.visualization.draw_geometries([circle_pcd, noisy_plane_pcd])
    circle_pts = np.array(circle_pcd.points)
    diameter = get_max_point_distance(circle_pts)
    diameter *= scale
    print("diameter: {}".format(diameter))
    keep_inds = []
    for ind, plane_point in enumerate(np.array(noisy_plane_pcd.points)):
        if np.linalg.norm(plane_point) > diameter / 2:
            keep_inds.append(ind)
    cropped_plane_pcd = plane_pcd.select_by_index(keep_inds, invert)
    return cropped_plane_pcd


def baseplate_plane_fitting(pcd, params):
    """
    Fit plane on baseplate for given pcd using specified params
    """
    # prep
    nominal_axis = params.nominal_axis
    plane_noise = params.plane_noise
    ransac_min_count = params.ransac_min_count
    ransac_num_iters = params.ransac_num_iters
    show = params.show
    aabb = pcd.get_axis_aligned_bounding_box()
    pcd.paint_uniform_color([0.5, 0.1, 0.1])

    # find principal axis of pcd
    # i.e axis that is closest to nominal axis
    pca = PCA(n_components=3)
    pca.fit(np.array(pcd.points))
    ref = [0.0, 0.0, 0.0]
    ref[nominal_axis] = 1.0
    axis_ind = np.argmax([np.abs(np.dot(c, ref)) for c in pca.components_])
    axis = pca.components_[axis_ind]
    axis /= np.linalg.norm(axis)
    if axis[nominal_axis] < 0:
        axis = -axis
    print("Axis: {}".format(axis))

    # crop scan to only have first 30% along axis
    min_extent = aabb.get_min_bound()
    min_extent = min_extent[nominal_axis]
    max_extent = aabb.get_max_bound()
    max_extent = max_extent[nominal_axis]
    cropping_inds = []
    for ind, pt in enumerate(np.array(pcd.points)):
        if pt[nominal_axis] < min_extent + 0.3 * (max_extent - min_extent):
            cropping_inds.append(ind)
    cropped_pcd = pcd.select_by_index(cropping_inds)
    print("Cropped pcd: {}".format(cropped_pcd))

    # segment plane out of cropped pcd
    coeffs, inds = cropped_pcd.segment_plane(plane_noise, ransac_min_count, ransac_num_iters)
    a, b, c, d = coeffs
    tmp_pcd = cropped_pcd.select_by_index(inds)
    tmp_pcd.paint_uniform_color([0.1, 0.1, 0.5])
    print("Plane coeffs: {}".format(coeffs))
    if show:
        o3d.visualization.draw_geometries([cropped_pcd, tmp_pcd])

    # sample plane and make plane_pcd
    plane_pts = []
    min_bounds = aabb.get_min_bound()
    max_bounds = aabb.get_max_bound()
    bbox_range = max_bounds - min_bounds
    for x in np.linspace(min_bounds[0], max_bounds[0], int(bbox_range[0] * 1e3)):
        for y in np.linspace(min_bounds[1], max_bounds[1], int(bbox_range[1] * 1e3)):
            z = (-d - (a * x) - (b * y)) / c
            plane_pts.append([x, y, z])
    plane_pcd = o3d.geometry.PointCloud()
    plane_pcd.points = o3d.utility.Vector3dVector(plane_pts)
    plane_pcd.normals = o3d.utility.Vector3dVector([axis] * len(plane_pts))
    plane_pcd.paint_uniform_color([0.1, 0.5, 0.1])
    if show:
        o3d.visualization.draw_geometries([cropped_pcd, plane_pcd])

    # move plane towards bevel, and find root gap boundary circle
    noisy_plane_pcd, plane_bbox, circle_pcd = get_circle_pcd(pcd, plane_pcd, axis, 0.005)
    if show:
        o3d.visualization.draw_geometries([circle_pcd, plane_bbox])

    # remove plane points that are inside the circle
    circle_center = plane_bbox.get_center()
    cropped_plane_pcd = crop_plane_with_circle(circle_pcd, circle_center, noisy_plane_pcd, plane_pcd)

    # move plane towards bevel, and find root gap boundary
    noisy_plane_pcd, plane_bbox, circle_pcd = get_circle_pcd(pcd, cropped_plane_pcd, axis, -0.001)
    if show:
        o3d.visualization.draw_geometries([circle_pcd, plane_bbox])

    # remove plane points that are inside the circle
    circle_center = plane_bbox.get_center()
    cropped_plane_pcd = crop_plane_with_circle(
        circle_pcd, circle_center, noisy_plane_pcd, cropped_plane_pcd, scale=0.9, invert=True
    )

    print("cropped_plane_pcd: {}".format(cropped_plane_pcd))
    if show:
        o3d.visualization.draw_geometries([cropped_plane_pcd, pcd])

    return cropped_plane_pcd + pcd


def local_test_madhavun():
    if len(sys.argv) == 2:
        dataset_dir = sys.argv[1]
        prefix = ""
    elif len(sys.argv) == 3:
        dataset_dir = sys.argv[1]
        prefix = sys.argv[2]
    else:
        dataset_dir = "/home/madhavun/data/Valmont/valmontCell/83_1080_2_1644337816367/"
        prefix = ""

    # mesh input cloud if required
    if len(prefix) > 0:
        _ = subprocess.call(
            "/home/madhavun/code/sandbox/cpp/hybridization/build/apps/examples/hybridization/hybridization {}".format(
                os.path.join(dataset_dir, prefix + "cloud_composite_normals.ply")
            ),
            shell=True,
        )
        os.system("mv mesh.ply {}".format(os.path.join(dataset_dir, prefix + "mesh.ply")))

    # load pcd
    filename = os.path.join(dataset_dir, prefix + "cloud_composite_normals.ply")
    pcd = o3d.io.read_point_cloud(filename)

    # plane fitting
    plane_fit_scan = baseplate_plane_fitting(pcd, BaseplatePlaneFittingParams())
    o3d.io.write_point_cloud(os.path.join(dataset_dir, prefix + "plane_fit_scan.ply"), plane_fit_scan)

    # mesh plane fit scan
    _ = subprocess.call(
        "/home/madhavun/code/sandbox/cpp/hybridization/build/apps/examples/hybridization/hybridization {}".format(
            os.path.join(dataset_dir, prefix + "plane_fit_scan.ply")
        ),
        shell=True,
    )
    os.system("mv mesh.ply {}".format(os.path.join(dataset_dir, prefix + "plane_fit_mesh.ply")))

    # open in meshlab
    os.system(
        "meshlab {} {} {}".format(
            os.path.join(dataset_dir, prefix + "plane_fit_scan.ply"),
            os.path.join(dataset_dir, prefix + "plane_fit_mesh.ply"),
            os.path.join(dataset_dir, prefix + "mesh.ply"),
        )
    )


if __name__ == "__main__":
    local_test_madhavun()
