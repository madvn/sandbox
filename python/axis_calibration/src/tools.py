import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R


def invert_tf(T):
    T[:3, :3] = np.linalg.inv(T[:3, :3])
    T[:3, -1] = np.matmul(-T[:3, :3], T[:3, -1])
    return T


def make_T_from_xyz_xyzw(xyz, xyzw, invert=False):
    rot_mat = R.from_quat(xyzw).as_matrix()
    return make_T_from_xyz_rotmat(xyz, rot_mat, invert)


def make_T_from_xyz_rpy(xyz, rpy, invert=False):
    rot_mat = R.from_euler("xyz", rpy).as_matrix()
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


def test():
    T = make_T_from_xyz_rpy(
        [-2.6967945e-03, 6.3908659e-03, 9.0318332e-05], [-0.0017533605, 0.0024056432, 0.0014615788], invert=True
    )
    print("translation: {}".format(T[:3, -1].tolist()))
    print("rotation xyzw: {}".format(R.from_matrix(T[:3, :3]).as_quat().tolist()))
    print("rotation rpy: {}".format(R.from_matrix(T[:3, :3]).as_euler("xyz").tolist()))


if __name__ == "__main__":
    test()
