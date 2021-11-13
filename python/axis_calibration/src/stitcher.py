import json
import os
import glob

import yaml

import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d


def invert_tf(T):
    T[:3, :3] = np.linalg.inv(T[:3, :3])
    T[:3, -1] = -T[:3, :3] @ T[:3, -1]
    return T


def make_T_from_xyz_xyzw(xyz, xyzw, invert=False):
    rot_mat = R.from_quat(xyzw).as_dcm()
    return make_T_from_xyz_rotmat(xyz, rot_mat, invert)


def make_T_from_xyz_rpy(xyz, rpy, invert=False):
    rot_mat = R.from_euler("xyz", rpy).as_dcm()
    return make_T_from_xyz_rotmat(xyz, rot_mat, invert)


def make_T_from_xyz_rotmat(xyz, rot_mat, invert=False):
    T = np.eye(4)
    T[:3, :3] = rot_mat
    T[:3, -1] = xyz
    if invert:
        return invert_tf(T)
    else:
        return T


def make_T_from_yaml(tf_file, invert=False):
    with open(tf_file) as f:
        tf = yaml.load(f, Loader=yaml.Loader)
    tf = tf["transform"]
    xyz = [tf["translation"][i] for i in "xyz"]
    if "x" in tf["rotation"]:
        xyzw = [tf["rotation"][i] for i in "xyzw"]
        return make_T_from_xyz_xyzw(xyz, xyzw, invert)
    elif "r" in tf["rotation"]:
        rpy = [tf["rotation"][i] for i in "rpy"]
        return make_T_from_xyz_rpy(xyz, rpy, invert)


def manual_stitch_clouds3(base_dir, section_cloud_files):
    composite_cloud = o3d.geometry.PointCloud()
    with open(os.path.join(base_dir, "transforms.json"), "r") as f:
        transforms = json.load(f)

    tf_S_to_C = np.array(transforms["SL_to_carriage_flange"])
    tf_C_to_B = make_T_from_xyz_rotmat(
        transforms["carriage_flange_to_positioner_base"]["translation"],
        transforms["carriage_flange_to_positioner_base"]["rotation"],
        invert=True,
    )

    for cloud_file in section_cloud_files:
        section_ind = int(cloud_file.split("_")[-1].split(".")[0])
        section_cloud = o3d.io.read_point_cloud(os.path.join(base_dir, cloud_file))
        section_cloud = section_cloud.transform(tf_S_to_C @ tf_C_to_B)

        section_tf = make_T_from_xyz_rotmat(
            transforms[str(section_ind)]["flange_to_tool0"]["translation"],
            transforms[str(section_ind)]["flange_to_tool0"]["rotation"],
        )

        section_cloud_tfed = section_cloud.transform(section_tf)  # to positioner frame
        section_cloud_tfed.paint_uniform_color(np.random.rand(3))
        composite_cloud += section_cloud_tfed

    o3d.visualization.draw_geometries(
        [
            composite_cloud,
            o3d.io.read_point_cloud(os.path.join(base_dir, "cloud_composite_normals.ply")).paint_uniform_color(
                [0.5, 0.5, 0.5]
            ),
        ]
    )
    o3d.io.write_point_cloud("ori_stitched_clouds.ply", composite_cloud)


def manual_stitch_clouds_Ben(base_dir, section_cloud_files, section_tf_files):
    # FRAMES:
    #     R - scanning robot base
    #     S - scanner primary
    #     C - Carriage flange
    #     B - Carriage positioner base
    #     A - Positioner theta=0

    assert len(section_cloud_files) == len(section_tf_files), "Mismatch in length of cloud files and tf files"
    composite_cloud = o3d.geometry.PointCloud()

    transforms = {}

    tf_S_to_C = make_T_from_yaml(os.path.join(base_dir, "primary_cam_to_carriage_flange.yaml"), invert=True)
    print("tf_S_to_C:\n", tf_S_to_C)
    transforms["SL_to_carriage_flange"] = tf_S_to_C.tolist()

    tf_C_to_B = make_T_from_yaml(os.path.join(base_dir, "carriage_flange_to_positioner_base.yaml"), invert=True)
    print("tf_C_to_B:\n", tf_C_to_B)
    transforms["carriage_flange_to_positioner_base"] = {}
    transforms["carriage_flange_to_positioner_base"]["translation"] = tf_C_to_B[:3, -1].tolist()
    transforms["carriage_flange_to_positioner_base"]["rotation"] = tf_C_to_B[:3, :3].tolist()

    for cloud_file in section_cloud_files:
        section_ind = int(cloud_file.split("_")[-1].split(".")[0])
        section_cloud = o3d.io.read_point_cloud(os.path.join(base_dir, cloud_file))
        section_cloud = section_cloud.transform(tf_S_to_C @ tf_C_to_B)

        section_tf = make_T_from_yaml(
            os.path.join(base_dir, f"positioner_base_to_positioner_tool0_{section_ind}.yaml"), invert=True
        )
        transforms[str(section_ind)] = {}
        transforms[str(section_ind)]["flange_to_tool0"] = {}
        transforms[str(section_ind)]["flange_to_tool0"]["translation"] = section_tf[:3, -1].tolist()
        transforms[str(section_ind)]["flange_to_tool0"]["rotation"] = section_tf[:3, :3].tolist()

        section_cloud_tfed = section_cloud.transform(section_tf)  # to positioner frame
        section_cloud_tfed.paint_uniform_color(np.random.rand(3))
        composite_cloud += section_cloud_tfed

    o3d.visualization.draw_geometries([composite_cloud])
    with open(os.path.join(base_dir, "transforms.json"), "w") as f:
        json.dump(transforms, f, indent=4)


def manual_stitch_clouds_Insik2(base_dir, section_cloud_files, section_tf_files):
    # FRAMES:
    #     R - scanning robot base
    #     S - scanner primary
    #     C - Carriage flange
    #     B - Carriage positioner base
    #     A - Positioner theta=0

    assert len(section_cloud_files) == len(section_tf_files), "Mismatch in length of cloud files and tf files"
    composite_cloud = o3d.geometry.PointCloud()

    transforms = {}

    tf_S_to_C = make_T_from_yaml(os.path.join(base_dir, "primary_cam_to_positioner_carriage.yaml"), invert=True)
    print("tf_S_to_C:\n", tf_S_to_C)
    transforms["SL_to_carriage_flange"] = tf_S_to_C.tolist()

    tf_C_to_B = make_T_from_yaml(os.path.join(base_dir, "positioner_carriage_to_positioner_base.yaml"), invert=True)
    print("tf_C_to_B:\n", tf_C_to_B)
    transforms["carriage_flange_to_positioner_base"] = {}
    transforms["carriage_flange_to_positioner_base"]["translation"] = tf_C_to_B[:3, -1].tolist()
    transforms["carriage_flange_to_positioner_base"]["rotation"] = tf_C_to_B[:3, :3].tolist()

    for cloud_file in section_cloud_files:
        section_ind = int(cloud_file.split("_")[-1].split(".")[0])
        section_cloud = o3d.io.read_point_cloud(os.path.join(base_dir, cloud_file))
        section_cloud = section_cloud.transform(tf_S_to_C @ tf_C_to_B)

        section_tf = make_T_from_yaml(
            os.path.join(base_dir, f"positioner_base_to_positioner_tool_{section_ind}.yaml"), invert=True
        )
        transforms[str(section_ind)] = {}
        transforms[str(section_ind)]["flange_to_tool0"] = {}
        transforms[str(section_ind)]["flange_to_tool0"]["translation"] = section_tf[:3, -1].tolist()
        transforms[str(section_ind)]["flange_to_tool0"]["rotation"] = section_tf[:3, :3].tolist()

        section_cloud_tfed = section_cloud.transform(section_tf)  # to positioner frame
        section_cloud_tfed.paint_uniform_color(np.random.rand(3))
        composite_cloud += section_cloud_tfed

    o3d.visualization.draw_geometries([composite_cloud])
    with open(os.path.join(base_dir, "transforms.json"), "w") as f:
        json.dump(transforms, f, indent=4)


def manual_stitch_clouds_Insik(base_dir, section_cloud_files, section_tf_files):
    # FRAMES:
    #     R - scanning robot base
    #     S - scanner primary
    #     C - Carriage flange
    #     B - Carriage positioner base
    #     A - Positioner theta=0

    assert len(section_cloud_files) == len(section_tf_files), "Mismatch in length of cloud files and tf files"
    composite_cloud = o3d.geometry.PointCloud()

    tf_S_to_C = make_T_from_yaml(os.path.join(base_dir, "primary_cam_to_positioner_carriage.yaml"), invert=True)
    print("tf_S_to_C:\n", tf_S_to_C)

    tf_C_to_B = make_T_from_yaml(os.path.join(base_dir, "positioner_carriage_to_positioner_base.yaml"), invert=True)
    print("tf_C_to_B:\n", tf_C_to_B)

    for cloud_file in section_cloud_files:
        section_ind = int(cloud_file.split("_")[-1].split(".")[0])
        section_cloud = o3d.io.read_point_cloud(os.path.join(base_dir, cloud_file))
        section_cloud = section_cloud.transform(tf_S_to_C @ tf_C_to_B)

        section_tf = make_T_from_yaml(
            os.path.join(base_dir, f"positioner_base_to_positioner_tool_{section_ind}.yaml"), invert=True
        )

        section_cloud_tfed = section_cloud.transform(section_tf)  # to positioner frame
        section_cloud_tfed.paint_uniform_color(np.random.rand(3))
        composite_cloud += section_cloud_tfed

    o3d.visualization.draw_geometries([composite_cloud])


def manual_stitch_clouds(base_dir, section_cloud_files, section_tf_files):
    # FRAMES:
    #     R - scanning robot base
    #     S - scanner primary
    #     C - Carriage flange
    #     B - Carriage positioner base
    #     A - Positioner theta=0

    assert len(section_cloud_files) == len(section_tf_files), "Mismatch in length of cloud files and tf files"
    composite_cloud = o3d.geometry.PointCloud()

    # tf_R_to_S = make_T_from_yaml(os.path.join(base_dir, ".yaml"))
    # tf_R_to_B = make_T_from_yaml(os.path.join(base_dir, "base_to_positioner_transform.yaml"))
    # tf_B_to_A = make_T_from_yaml(os.path.join(base_dir, "positioner_base_to_Positioner_tool.yaml"))

    # tf_B_to_A = make_T_from_xyz_xyzw([0.548, 0.000, 0.914], [0.685, 0.174, 0.685, 0.174])
    # tf_Rflange_to_Rtool0 = make_T_from_xyz_xyzw([0., 0., 0.], [0.707, 0.000, 0.707, -0.000])

    tf_S_to_Rflange = make_T_from_yaml("../calibration/calibration/robot_base_to_sl_tf.yaml", invert=True)
    tf_Rflange_to_world = make_T_from_yaml(os.path.join(base_dir, "scanner_pose.yaml"))
    tf_world_to_C = make_T_from_xyz_xyzw([0.385, 2.090, 0.444], [0.000, 0.000, 0.000, 1.000], invert=True)
    tf_C_to_B = make_T_from_yaml("../calibration/zone_1/positioner_calibration.yaml")
    tf_B_to_A = make_T_from_xyz_xyzw([0.548, 0.000, 0.914], [0.685, 0.174, 0.685, 0.174], invert=True)

    tf_S_to_world = tf_S_to_Rflange @ tf_Rflange_to_world
    print("tf_S_to_world:\n", tf_S_to_world)
    print("tf_world_to_C:\n", tf_world_to_C)
    tf_S_to_C = tf_S_to_world @ tf_world_to_C
    print("tf_S_to_C:\n", tf_S_to_C)
    tf_S_to_B = tf_S_to_C @ tf_C_to_B
    print("tf_S_to_B:\n", tf_S_to_B)
    section_tf_0 = tf_B_to_A @ tf_S_to_B
    print("section_tf_0:\n", section_tf_0)

    for cloud_file in section_cloud_files:
        section_ind = int(cloud_file.split("_")[-1].split(".")[0])  # account for positioenr rot
        section_theta = -np.pi + section_ind * (2 * np.pi / len(section_cloud_files))
        tf_theta = np.eye(4)
        tf_theta[:3, :3] = R.from_rotvec([0, 0, section_theta]).as_dcm()
        section_tf = section_tf_0 @ tf_theta

        section_cloud = o3d.io.read_point_cloud(os.path.join(base_dir, cloud_file))
        section_cloud_tfed = section_cloud.transform(section_tf)  # to positioner frame
        section_cloud_tfed.paint_uniform_color(np.random.rand(3))
        composite_cloud += section_cloud_tfed

    o3d.visualization.draw_geometries([composite_cloud])


def make_section_clouds_Insik(base_dir, transformed_cloud_files, section_tf_files):
    assert len(transformed_cloud_files) == len(section_tf_files), "Mismatch in length of cloud files and tf files"
    composite_cloud = o3d.geometry.PointCloud()

    for cloud_file, tf_file in zip(transformed_cloud_files, section_tf_files):
        section_ind = int(cloud_file.split("_")[-1][:-4])
        section_cloud = o3d.io.read_point_cloud(os.path.join(base_dir, cloud_file))
        tf = make_T_from_yaml(os.path.join(base_dir, tf_file))
        section_cloud_tfed = section_cloud.transform(invert_tf(tf))
        section_cloud_tfed.paint_uniform_color(np.random.rand(3))
        # o3d.io.write_point_cloud(os.path.join(base_dir, "tf_" + cloud_file), section_cloud_tfed)
        o3d.io.write_point_cloud(os.path.join(base_dir, "section_cloud_{}.ply".format(section_ind)), section_cloud_tfed)
        composite_cloud += section_cloud_tfed

    o3d.visualization.draw_geometries([composite_cloud])


def stitch_section_clouds_Ben(base_dir, section_cloud_files, section_tf_files):
    assert len(section_cloud_files) == len(section_tf_files), "Mismatch in length of cloud files and tf files"
    composite_cloud = o3d.geometry.PointCloud()

    for cloud_file, tf_file in zip(section_cloud_files, section_tf_files):
        section_cloud = o3d.io.read_point_cloud(os.path.join(base_dir, cloud_file))
        tf = make_T_from_yaml(os.path.join(base_dir, tf_file))
        section_cloud_tfed = section_cloud.transform(tf)
        section_cloud_tfed.paint_uniform_color(np.random.rand(3))
        # o3d.io.write_point_cloud(os.path.join(base_dir, "tf_" + cloud_file), section_cloud_tfed)
        o3d.io.write_point_cloud(os.path.join(base_dir, "tf_" + cloud_file), section_cloud_tfed)
        composite_cloud += section_cloud_tfed

    o3d.visualization.draw_geometries([composite_cloud])


if __name__ == "__main__":
    base_dir = "/home/madhavun/data/Valmont/valmontCell/3_30_2_1635454531169"
    section_cloud_files = [
        "section_cloud_1.ply",
        "section_cloud_2.ply",
        "section_cloud_3.ply",
        "section_cloud_4.ply",
        "section_cloud_0.ply",
        "section_cloud_5.ply",
        "section_cloud_6.ply",
        "section_cloud_7.ply",
        "section_cloud_8.ply",
        "section_cloud_9.ply",
    ]
    section_tf_files = [
        "section_cloud_tf_0.yaml",
        "section_cloud_tf_65536.yaml",
        "section_cloud_tf_131072.yaml",
        "section_cloud_tf_196608.yaml",
        "section_cloud_tf_262144.yaml",
        "section_cloud_tf_327680.yaml",
        "section_cloud_tf_393216.yaml",
        "section_cloud_tf_458752.yaml",
        "section_cloud_tf_524288.yaml",
        "section_cloud_tf_589824.yaml",
    ]

    # base_dir = "/home/madhavun/data/Valmont/valmontCell/8_213_2_1636067775852/"
    base_dir = "/home/madhavun/data/Valmont/valmontCell/8_258_2_1636237067045/"
    section_cloud_files = [
        "section_cloud_0.ply",
        "section_cloud_1.ply",
        "section_cloud_2.ply",
        "section_cloud_3.ply",
        "section_cloud_4.ply",
        "section_cloud_5.ply",
        "section_cloud_6.ply",
        "section_cloud_7.ply",
        "section_cloud_8.ply",
        "section_cloud_9.ply",
        "section_cloud_10.ply",
        "section_cloud_11.ply",
        "section_cloud_12.ply",
        "section_cloud_13.ply",
        "section_cloud_14.ply",
        "section_cloud_15.ply",
    ]
    section_tf_files = [
        "section_cloud_tf_0.yaml",
        "section_cloud_tf_65536.yaml",
        "section_cloud_tf_131072.yaml",
        "section_cloud_tf_196608.yaml",
        "section_cloud_tf_262144.yaml",
        "section_cloud_tf_327680.yaml",
        "section_cloud_tf_393216.yaml",
        "section_cloud_tf_458752.yaml",
        "section_cloud_tf_524288.yaml",
        "section_cloud_tf_589824.yaml",
        "section_cloud_tf_655360.yaml",
        "section_cloud_tf_720896.yaml",
        "section_cloud_tf_786432.yaml",
        "section_cloud_tf_851968.yaml",
        "section_cloud_tf_917504.yaml",
        "section_cloud_tf_983040.yaml",
    ]

    # base_dir = "/home/madhavun/data/Valmont/valmontCell/y_offset_0.02"
    # section_cloud_files = [
    #     "section_cloud_0.ply",
    #     "section_cloud_1.ply",
    #     "section_cloud_2.ply",
    #     "section_cloud_3.ply",
    #     "section_cloud_4.ply",
    #     "section_cloud_5.ply",
    #     "section_cloud_6.ply",
    #     "section_cloud_7.ply",
    #     "section_cloud_8.ply",
    #     "section_cloud_9.ply",
    # ]
    # section_tf_files = [
    #     "section_cloud_tf_0.yaml",
    #     "section_cloud_tf_65536.yaml",
    #     "section_cloud_tf_131072.yaml",
    #     "section_cloud_tf_196608.yaml",
    #     "section_cloud_tf_262144.yaml",
    #     "section_cloud_tf_327680.yaml",
    #     "section_cloud_tf_393216.yaml",
    #     "section_cloud_tf_458752.yaml",
    #     "section_cloud_tf_524288.yaml",
    #     "section_cloud_tf_589824.yaml",
    # ]

    # base_dir = "/home/madhavun/data/Valmont/valmontCell/22_295_2_1636564143165/"
    # section_cloud_files = [
    #     "section_cloud_0.ply",
    #     "section_cloud_1.ply",
    #     "section_cloud_2.ply",
    #     "section_cloud_3.ply",
    #     "section_cloud_4.ply",
    #     "section_cloud_5.ply",
    #     "section_cloud_6.ply",
    #     "section_cloud_7.ply",
    #     "section_cloud_8.ply",
    #     "section_cloud_9.ply",
    # ]
    # section_tf_files = [
    #     "section_cloud_tf_0.yaml",
    #     "section_cloud_tf_65536.yaml",
    #     "section_cloud_tf_131072.yaml",
    #     "section_cloud_tf_196608.yaml",
    #     "section_cloud_tf_262144.yaml",
    #     "section_cloud_tf_327680.yaml",
    #     "section_cloud_tf_393216.yaml",
    #     "section_cloud_tf_458752.yaml",
    #     "section_cloud_tf_524288.yaml",
    #     "section_cloud_tf_589824.yaml",
    # ]

    base_dir = "/home/madhavun/data/Valmont/valmontCell/z_offsets/4_24_2_1636703828840"
    # base_dir = "/home/madhavun/data/Valmont/valmontCell/z_offsets/4_27_2_1636708143345"
    transformed_cloud_files = [
        "transformed_cloud_0.ply",
        "transformed_cloud_1.ply",
        "transformed_cloud_2.ply",
        "transformed_cloud_3.ply",
        "transformed_cloud_4.ply",
        "transformed_cloud_5.ply",
        "transformed_cloud_6.ply",
        "transformed_cloud_7.ply",
        "transformed_cloud_8.ply",
        "transformed_cloud_9.ply",
        "transformed_cloud_10.ply",
        "transformed_cloud_11.ply",
        "transformed_cloud_12.ply",
        "transformed_cloud_13.ply",
        "transformed_cloud_14.ply",
        "transformed_cloud_15.ply",
        "transformed_cloud_16.ply",
        "transformed_cloud_17.ply",
        "transformed_cloud_18.ply",
        "transformed_cloud_19.ply",
    ]
    section_cloud_files = [
        "section_cloud_0.ply",
        "section_cloud_1.ply",
        "section_cloud_2.ply",
        "section_cloud_3.ply",
        "section_cloud_4.ply",
        "section_cloud_5.ply",
        "section_cloud_6.ply",
        "section_cloud_7.ply",
        "section_cloud_8.ply",
        "section_cloud_9.ply",
        "section_cloud_10.ply",
        "section_cloud_11.ply",
        "section_cloud_12.ply",
        "section_cloud_13.ply",
        "section_cloud_14.ply",
        "section_cloud_15.ply",
        "section_cloud_16.ply",
        "section_cloud_17.ply",
        "section_cloud_18.ply",
        "section_cloud_19.ply",
    ]
    section_tf_files = [
        "section_cloud_tf_0.yaml",
        "section_cloud_tf_65536.yaml",
        "section_cloud_tf_131072.yaml",
        "section_cloud_tf_196608.yaml",
        "section_cloud_tf_262144.yaml",
        "section_cloud_tf_327680.yaml",
        "section_cloud_tf_393216.yaml",
        "section_cloud_tf_458752.yaml",
        "section_cloud_tf_524288.yaml",
        "section_cloud_tf_589824.yaml",
        "section_cloud_tf_655360.yaml",
        "section_cloud_tf_720896.yaml",
        "section_cloud_tf_786432.yaml",
        "section_cloud_tf_851968.yaml",
        "section_cloud_tf_917504.yaml",
        "section_cloud_tf_983040.yaml",
        "section_cloud_tf_1048576.yaml",
        "section_cloud_tf_1114112.yaml",
        "section_cloud_tf_1179648.yaml",
        "section_cloud_tf_1245184.yaml",
    ]

    # T = make_T_from_yaml("/home/madhavun/data/Valmont/valmontCell/3_30_2_1635454531169/section_cloud_tf_65536.yaml")
    # print(T)

    # make_section_clouds_Insik(base_dir, transformed_cloud_files, section_tf_files)
    # stitch_section_clouds_Ben(base_dir, section_cloud_files, section_tf_files)

    # manual_stitch_clouds_Ben(base_dir, section_cloud_files, section_tf_files)
    manual_stitch_clouds_Insik(base_dir, section_cloud_files, section_tf_files)
    # manual_stitch_clouds3(base_dir, section_cloud_files)
