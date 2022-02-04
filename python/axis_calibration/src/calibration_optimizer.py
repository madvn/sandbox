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

from tools import *


def make_section_clouds_from_transformed_clouds(base_dir):
    """
    read all transformed clouds, apply inverse of section_tf to them, and save as section_cloud
    """
    print("Making section clouds from transformed clouds")
    # find and sort transformed cloud files
    transformed_cloud_files = glob.glob(os.path.join(base_dir, "transformed_cloud_*.ply"))
    transformed_cloud_files = sorted(transformed_cloud_files, key=lambda f: int(f.split("/")[-1].split("_")[-1][:-4]))

    # find and sort section tf files
    section_tf_files = glob.glob(os.path.join(base_dir, "section_cloud_tf_*.yaml"))
    section_tf_files = sorted(section_tf_files, key=lambda f: int(f.split("/")[-1].split("_")[-1][:-5]))
    section_tf_files = section_tf_files[1::2]

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


def read_transforms(base_dir):
    """
    Read all necessary transform for axis calibration
    """
    section_cloud_files = glob.glob(os.path.join(base_dir, "section_cloud_*.ply"))
    if len(section_cloud_files) == 0:
        make_section_clouds_from_transformed_clouds(base_dir)
        section_cloud_files = glob.glob(os.path.join(base_dir, "section_cloud_*.ply"))
    section_cloud_files.sort()

    transforms = {}

    for cloud_file in section_cloud_files:
        section_ind = int(cloud_file.split("_")[-1].split(".")[0])
        transforms[str(section_ind)] = {}

        tf_S_to_C = make_T_from_yaml(
            os.path.join(base_dir, f"primary_cam_to_positioner_carriage_{section_ind}.yaml"), invert=True
        )
        transforms[str(section_ind)]["SL_to_carriage_flange"] = tf_S_to_C.tolist()

        tf_C_to_B = make_T_from_yaml(
            os.path.join(base_dir, f"positioner_carriage_to_positioner_base_{section_ind}.yaml"), invert=True
        )
        transforms[str(section_ind)]["carriage_flange_to_positioner_base"] = {}
        transforms[str(section_ind)]["carriage_flange_to_positioner_base"]["translation"] = tf_C_to_B[:3, -1].tolist()
        transforms[str(section_ind)]["carriage_flange_to_positioner_base"]["rotation"] = tf_C_to_B[:3, :3].tolist()

        section_tf = make_T_from_yaml(
            os.path.join(base_dir, f"positioner_base_to_positioner_tool_{section_ind}.yaml"), invert=True
        )
        transforms[str(section_ind)]["positioner_base_to_tool0"] = {}
        transforms[str(section_ind)]["positioner_base_to_tool0"]["translation"] = section_tf[:3, -1].tolist()
        transforms[str(section_ind)]["positioner_base_to_tool0"]["rotation"] = section_tf[:3, :3].tolist()

    return transforms


def optimize_calibration(pts_data, transforms, num_epochs, vars_to_train, DEVICE):
    os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
    os.environ["CUDA_VISIBLE_DEVICES"] = DEVICE[-1]
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True

    C_to_B_t = transforms["0"]["carriage_flange_to_positioner_base"]["translation"]
    C_to_B_r = R.from_matrix(transforms["0"]["carriage_flange_to_positioner_base"]["rotation"]).as_euler("xyz")

    #############################################
    # Build computational graph for optimization
    #############################################
    with tf.device(DEVICE):
        pts1_pl = tf.placeholder(tf.float32, [None, 3])
        pts2_pl = tf.placeholder(tf.float32, [None, 3])

        C_to_B_tx_trainable = tf.Variable(C_to_B_t[0], dtype=tf.float32, trainable=True)
        C_to_B_ty_trainable = tf.Variable(C_to_B_t[1], dtype=tf.float32, trainable=True)
        C_to_B_tz_trainable = tf.Variable(C_to_B_t[2], dtype=tf.float32, trainable=True)
        C_to_B_rotx_trainable = tf.Variable(C_to_B_r[0], dtype=tf.float32, trainable=True)
        C_to_B_roty_trainable = tf.Variable(C_to_B_r[1], dtype=tf.float32, trainable=True)
        C_to_B_rotz_trainable = tf.Variable(C_to_B_r[2], dtype=tf.float32, trainable=True)

        H_t = tf.constant(C_to_B_t, dtype=tf.float32)
        H_r = tf.constant(C_to_B_r, dtype=tf.float32)

        rot_matz = tf.stack(
            (
                (tf.cos(C_to_B_rotz_trainable), -tf.sin(C_to_B_rotz_trainable), 0),
                (tf.sin(C_to_B_rotz_trainable), tf.cos(C_to_B_rotz_trainable), 0),
                (0, 0, 1),
            ),
            axis=0,
        )
        rot_maty = tf.stack(
            (
                (tf.cos(C_to_B_roty_trainable), 0, tf.sin(C_to_B_roty_trainable)),
                (0, 1, 0),
                (-tf.sin(C_to_B_roty_trainable), 0, tf.cos(C_to_B_roty_trainable)),
            ),
            axis=0,
        )
        rot_matx = tf.stack(
            (
                (1, 0, 0),
                (0, tf.cos(C_to_B_rotx_trainable), -tf.sin(C_to_B_rotx_trainable)),
                (0, tf.sin(C_to_B_rotx_trainable), tf.cos(C_to_B_rotx_trainable)),
            ),
            axis=0,
        )
        heye_r = tf.matmul(tf.matmul(rot_matz, rot_maty), rot_matx)

        # transform points to positioner base
        pts1_positioner_base = tf.transpose(tf.matmul(heye_r, tf.transpose(pts1_pl))) + [
            C_to_B_tx_trainable,
            C_to_B_ty_trainable,
            C_to_B_tz_trainable,
        ]
        pts2_positioner_base = tf.transpose(tf.matmul(heye_r, tf.transpose(pts2_pl))) + [
            C_to_B_tx_trainable,
            C_to_B_ty_trainable,
            C_to_B_tz_trainable,
        ]

        # pl to transform to positioner tool0
        B_to_A_1_t_pl = tf.placeholder(tf.float32, shape=(3,))
        B_to_A_1_r_pl = tf.placeholder(tf.float32, shape=(3, 3))
        B_to_A_2_t_pl = tf.placeholder(tf.float32, shape=(3,))
        B_to_A_2_r_pl = tf.placeholder(tf.float32, shape=(3, 3))

        # transform to tool0
        pts1_M = tf.transpose(tf.matmul(B_to_A_1_r_pl, tf.transpose(pts1_positioner_base))) + B_to_A_1_t_pl
        pts2_M = tf.transpose(tf.matmul(B_to_A_2_r_pl, tf.transpose(pts2_positioner_base))) + B_to_A_2_t_pl

        # main loss -- 2-way chamfer distance
        num_point1 = tf.placeholder(tf.int32, (None))
        num_point2 = tf.placeholder(tf.int32, (None))
        num_features = tf.placeholder(tf.int32, (None))
        grads_pl = tf.placeholder(tf.float32, (None,))
        expanded_pts1_M = tf.tile(pts1_M, (num_point2, 1))
        expanded_pts2_M = tf.reshape(tf.tile(tf.expand_dims(pts2_M, 1), (1, num_point1, 1)), (-1, num_features))
        distances = tf.norm(expanded_pts1_M - expanded_pts2_M, axis=1)
        distances = tf.reshape(distances, (num_point2, num_point1))
        loss1 = tf.reduce_sum(tf.reduce_min(distances, 0))
        loss2 = tf.reduce_sum(tf.reduce_min(distances, 1))
        mean_dists = (tf.reduce_mean(tf.reduce_min(distances, 0)) + tf.reduce_mean(tf.reduce_min(distances, 1))) / 2

        # auxiliary loss -- stay close to current axis
        auxil_loss1 = tf.reduce_sum((H_t - [C_to_B_tx_trainable, C_to_B_ty_trainable, C_to_B_tz_trainable]) ** 2)
        auxil_loss2 = tf.reduce_sum((H_r - (C_to_B_rotx_trainable, C_to_B_roty_trainable, C_to_B_rotz_trainable)) ** 2)
        optim_loss = (loss1 + loss2) / 80
        auxil_loss = (auxil_loss1 + auxil_loss2) * 10 ** 2 / 10
        loss = optim_loss  # + auxil_loss

        # gradient estimation and update ops
        gradient_holders = []
        tvars = tf.trainable_variables()
        for idx, var in enumerate(tvars):
            placeholder = tf.placeholder(tf.float32, name=str(idx) + "_holder")
            gradient_holders.append(placeholder)
        grad_op = tf.gradients(loss, tf.trainable_variables())
        optimizer = tf.train.AdamOptimizer(learning_rate=1e-4)
        train_op = optimizer.apply_gradients(zip(gradient_holders, tvars))

    with tf.Session() as sess:
        #############################################
        # Prep and data before optimization
        #############################################
        sess.run(tf.compat.v1.global_variables_initializer())

        # before optim
        pcd_colors = [np.random.rand(3) for _ in range(len(pts_data))]
        before_rpy = sess.run([C_to_B_rotx_trainable, C_to_B_roty_trainable, C_to_B_rotz_trainable])
        before_t = sess.run([[C_to_B_tx_trainable, C_to_B_ty_trainable, C_to_B_tz_trainable]])
        pcds = []
        for ind1, pts in enumerate(pts_data):
            pts1_m = sess.run(
                [pts1_M],
                feed_dict={
                    pts1_pl: pts,
                    B_to_A_1_t_pl: transforms[str(ind1)]["positioner_base_to_tool0"]["translation"],
                    B_to_A_1_r_pl: transforms[str(ind1)]["positioner_base_to_tool0"]["rotation"],
                },
            )
            pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts1_m[0])).paint_uniform_color(pcd_colors[ind1])
            pcds.append(pcd1)
        # o3d.visualization.draw_geometries(pcds)
        del pcd1, pcds

        #############################################
        # Optimization
        #############################################
        ls = []
        als = []

        def run_training_epochs(n_epochs, train_mask):
            epoch = 0
            while epoch < n_epochs:
                l, al = 0.0, 0.0
                grad_buffer = sess.run(tf.trainable_variables())
                for ix, grad in enumerate(grad_buffer):
                    grad_buffer[ix] = grad * 0

                # print("Running epoch {} with pcds {} and {}".format(epoch + 1, ind1, ind2))
                num_updates_per_epoch = 0
                for ind1 in range(len(pts_data)):
                    ind2 = (ind1 + 1) % len(pts_data)
                    _l, _al, dLdH = sess.run(
                        [optim_loss, auxil_loss, grad_op],
                        feed_dict={
                            pts1_pl: pts_data[ind1],
                            pts2_pl: pts_data[ind2],
                            num_point1: len(pts_data[ind1]),
                            num_point2: len(pts_data[ind2]),
                            num_features: 3,
                            B_to_A_1_t_pl: transforms[str(ind1)]["positioner_base_to_tool0"]["translation"],
                            B_to_A_1_r_pl: transforms[str(ind1)]["positioner_base_to_tool0"]["rotation"],
                            B_to_A_2_t_pl: transforms[str(ind2)]["positioner_base_to_tool0"]["translation"],
                            B_to_A_2_r_pl: transforms[str(ind2)]["positioner_base_to_tool0"]["rotation"],
                        },
                    )
                    l += _l
                    al += _al
                    num_updates_per_epoch += 1

                    # cumulating gradients
                    for ix, grad in enumerate(dLdH):
                        grad_buffer[ix] += grad * train_mask[ix]

                # applying gradients
                epoch += 1
                for ix, grad in enumerate(dLdH):
                    grad_buffer[ix] /= num_updates_per_epoch
                feed_dict = dict(zip(gradient_holders, grad_buffer))
                _ = sess.run(train_op, feed_dict=feed_dict)
                ls.append(l / num_updates_per_epoch)
                als.append(al / num_updates_per_epoch)

                print("End of epoch {}/{} | Mean loss this epoch = {}, {}".format(epoch, n_epochs, ls[-1], als[-1]))

        for n_epochs, train_mask in zip(num_epochs, vars_to_train):
            run_training_epochs(n_epochs, train_mask)

        result_xyz = list(sess.run([[C_to_B_tx_trainable, C_to_B_ty_trainable, C_to_B_tz_trainable]])[0])
        result_rpy = list(sess.run([C_to_B_rotx_trainable, C_to_B_roty_trainable, C_to_B_rotz_trainable]))

        res = ""
        res += "\n\n" + "##" * 5 + " Before " + "##" * 5 + "\n"
        res += "Rot_rpy:{}".format(before_rpy) + "\n"
        res += "Translation:{}".format(list(before_t[0])) + "\n"
        res += "\n\n" + "##" * 5 + " After " + "##" * 5 + "\n"
        res += "Rot_rpy:{}".format(result_rpy) + "\n"
        res += "Translation:{}".format(result_xyz) + "\n"

        #############################################
        # Stitch pcds after optimization and show
        #############################################
        new_stitched_pcd = o3d.geometry.PointCloud()
        for ind1, pts in enumerate(pts_data):
            pts1_m = sess.run(
                [pts1_M],
                feed_dict={
                    pts1_pl: pts,
                    B_to_A_1_t_pl: transforms[str(ind1)]["positioner_base_to_tool0"]["translation"],
                    B_to_A_1_r_pl: transforms[str(ind1)]["positioner_base_to_tool0"]["rotation"],
                },
            )
            pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts1_m[0]))
            pcd1.paint_uniform_color(np.random.rand(3))  # [0.8703, 0.3481, 0.3481]
            new_stitched_pcd += pcd1

    T = make_T_from_xyz_rpy(result_xyz, result_rpy, invert=False)

    print("\nInverted tf to use i.e. final results")
    res_txt = ""
    res_txt += "translation: {}\n".format(T[:3, -1].tolist())
    res_txt += "rotation xyzw: {}\n".format(R.from_matrix(T[:3, :3]).as_quat().tolist())
    res_txt += "rotation rpy: {}".format(R.from_matrix(T[:3, :3]).as_euler("xyz").tolist())
    print(res_txt)

    return ls, als, invert_tf(T), new_stitched_pcd


def main(base_dir, output_dir, num_epochs, viz):
    #############################################
    # setup
    #############################################
    DEVICE = "/gpu:0"
    num_epochs = [num_epochs]  # /2, num_epochs/2]
    vars_to_train = [[1, 1, 1, 0, 0, 0]]  # , [0, 0, 1, 0, 0, 0]]  # ,[0, 1, 0, 0, 0, 0]]  # [x,y,z,r,p,y]

    #############################################
    # load data and transforms
    #############################################
    # base_dir = "/home/madhavun/data/Valmont/valmontCell/8_258_2_1636237067045/"
    # section_cloud_files = glob.glob(os.path.join(base_dir, "section_cloud_*.ply"))
    # section_cloud_files.sort()
    # print("\n".join(section_cloud_files))

    # load in transforms for all scans i.e. base to tool0_theta for each scan
    transforms = read_transforms(base_dir)

    section_cloud_files = glob.glob(os.path.join(base_dir, "section_cloud_*.ply"))
    section_cloud_files = sorted(section_cloud_files, key=lambda f: int(f.split("/")[-1].split("_")[-1][:-4]))
    print("\n".join(section_cloud_files))

    # load scans and transform to carriage flange
    pts_data = []
    pcds = []
    for cloud_file in section_cloud_files:
        section_ind = int(cloud_file.split("_")[-1].split(".")[0])
        c = np.random.rand(3)
        c /= np.linalg.norm(c)
        pcd = o3d.io.read_point_cloud(os.path.join(cloud_file)).paint_uniform_color(c)
        pcd = pcd.voxel_down_sample(0.005)
        S_to_C = transforms[str(section_ind)]["SL_to_carriage_flange"]
        pcd = pcd.transform(S_to_C)
        pcds.append(pcd)
        pts_data.append(np.array(pcd.points))
    del pcd
    print("Loaded {} point clouds".format(len(pts_data)))
    if viz:
        print("Showing all section scans in carriage flange frame")
        o3d.visualization.draw_geometries(pcds)

    #############################################
    # Registration-based axis calibration
    #############################################
    ls, als, calibrated_T, new_stitched_pcd = optimize_calibration(
        pts_data, transforms, num_epochs, vars_to_train, DEVICE
    )

    #############################################
    # Log results after optimization
    #############################################
    o3d.io.write_point_cloud(os.path.join(output_dir, "new_stitched_pcd.ply"), new_stitched_pcd)

    fig, ax1 = plt.subplots(figsize=[4, 3])
    color = "tab:red"
    # ax1.plot(smoothen(ls, 20), label="Optim Loss", color=color, alpha=0.5)
    ax1.plot(ls, label="Optim Loss", color=color, alpha=0.5)
    ax1.set_ylabel("Optim loss", color=color)
    ax1.tick_params(axis="y", labelcolor=color)

    ax2 = ax1.twinx()
    color = "tab:blue"
    ax2.plot(als, label="Auxil Loss", color=color, alpha=0.5)
    ax2.set_ylabel("Auxiliary Loss", color=color)
    ax2.tick_params(axis="y", labelcolor=color)
    ax2.set_xlabel("Time")

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "optim.png"))
    if viz:
        plt.show()

    print("\nInverted tf to use i.e. final results")
    res_txt = ""
    res_txt += "translation: {}\n".format(calibrated_T[:3, -1].tolist())
    res_txt += "rotation xyzw: {}\n".format(R.from_matrix(calibrated_T[:3, :3]).as_quat().tolist())
    res_txt += "rotation rpy: {}".format(R.from_matrix(calibrated_T[:3, :3]).as_euler("xyz").tolist())
    print(res_txt)
    with open(os.path.join(output_dir, "results.txt"), "w") as f:
        f.write(res_txt)

    if viz:
        o3d.visualization.draw_geometries([new_stitched_pcd])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--data-dir",
        type=str,
        required=True,
        help="path to dataset",
        # default="/home/madhavun/data/Valmont/valmontCell/8_258_2_1636237067045/",
        # default="/home/madhavun/data/Valmont/valmontCell/z_offsets/4_24_2_1636703828840",
        # default = "/home/madhavun/data/Valmont/valmontCell/z_offsets/4_27_2_1636708143345",
        dest="base_dir",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=str,
        required=False,
        help="path to store results in",
        default="./",
        dest="output_dir",
    )
    parser.add_argument(
        "-e",
        "--num-epochs",
        type=int,
        required=False,
        help="number of epochs",
        default=150,
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
    main(args.base_dir, args.output_dir, args.num_epochs, args.viz)
