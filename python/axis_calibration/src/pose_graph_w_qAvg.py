import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats.mstats import gmean

voxel_size = 0.005
max_correspondence_distance_coarse = voxel_size * 3
max_correspondence_distance_fine = voxel_size * 1.5


def load_point_clouds(voxel_size=0.0):
    pcds = []
    for i in range(16):
        pcd = o3d.io.read_point_cloud(
            "/home/madhavun/data/Valmont/valmontCell/8_213_2_1636067775852/tf_section_cloud_{}.ply".format(i)
        )
        # pcd = o3d.io.read_point_cloud("/home/madhavun/data/Valmont/valmontCell/3_30_2_1635454531169/tf_section_cloud_{}.ply".format(i))
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd_down.estimate_normals()
        pcds.append(pcd_down)
    return pcds


def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_correspondence_distance_coarse,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    icp_fine = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine, icp_fine.transformation
    )
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse, max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(pcds[source_id], pcds[target_id])
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        source_id, target_id, transformation_icp, information_icp, uncertain=False
                    )
                )
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(
                        source_id, target_id, transformation_icp, information_icp, uncertain=True
                    )
                )
    return pose_graph


if __name__ == "__main__":

    # o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    pcds_down = load_point_clouds(voxel_size)
    o3d.visualization.draw_geometries(pcds_down)

    print("Full registration ...")
    pose_graph = full_registration(pcds_down, max_correspondence_distance_coarse, max_correspondence_distance_fine)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine, edge_prune_threshold=0.25, reference_node=0
    )
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option,
    )

    print("Transform points and display")
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    o3d.visualization.draw_geometries(pcds_down)

    print("Make a combined point cloud")
    pcds = load_point_clouds(voxel_size)
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    o3d.io.write_point_cloud("multiway_registration.ply", pcd_combined_down)
    o3d.visualization.draw_geometries([pcd_combined_down])

    print("---------------------")
    for i in range(len(pcds)):
        print(pose_graph.nodes[i].pose)

    print("---------------------")
    print("Make a combined point cloud with single T")
    T = np.eye(4)
    q = np.zeros((4, len(pcds)))
    p = np.zeros(3)
    for i in range(len(pcds)):
        p += pose_graph.nodes[i].pose[:3, -1]
        q[:, i] = R.from_dcm(pose_graph.nodes[i].pose[:3, :3]).as_quat()
    p /= len(pcds)
    T[:3, -1] = p
    qqT = np.matmul(q, np.transpose(q))
    eigen_vals, eigen_vecs = np.linalg.eig(qqT)
    q_avg = eigen_vecs[np.argmax(eigen_vals)]
    print(p)
    print(q_avg)
    # T[:3,:3] =  R.from_quat(q_avg).as_dcm()
    # T[:3,:3] =  R.from_quat(gmean(q, axis=1)).as_dcm()
    T[:3, :3] = R.from_quat(q.T).mean().as_dcm()
    print(T)
    pcds = load_point_clouds(voxel_size)
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(T)
        pcd_combined += pcds[point_id]
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    o3d.io.write_point_cloud("multiway_registration_singleT.ply", pcd_combined_down)
    print("\ntf to use ")
    # from tools import make_T_from_xyz_rpy
    # T = invert_tf(result_xyz, result_rpy, invert=True)
    print("translation: {}".format(T[:3, -1].tolist()))
    print("rotation xyzw: {}".format(R.from_dcm(T[:3, :3]).as_quat().tolist()))
    print("rotation rpy: {}".format(R.from_dcm(T[:3, :3]).as_euler("xyz").tolist()))

    o3d.visualization.draw_geometries([pcd_combined_down])

    print("---------------------")
    print("\ntf[1]")
    T0 = pose_graph.nodes[1].pose
    # from tools import make_T_from_xyz_rpy
    # T = invert_tf(result_xyz, result_rpy, invert=True)
    print("translation: {}".format(T0[:3, -1].tolist()))
    print("rotation xyzw: {}".format(R.from_dcm(T0[:3, :3]).as_quat().tolist()))
    print("rotation rpy: {}".format(R.from_dcm(T0[:3, :3]).as_euler("xyz").tolist()))
