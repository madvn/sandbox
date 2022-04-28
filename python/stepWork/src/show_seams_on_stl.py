import os
import glob
import open3d as o3d

def show_seams_on_stl(stl_dir, seam_dir):
    composite_mesh = o3d.geometry.TriangleMesh()
    surfaces = []
    for stl_file in glob.glob(os.path.join(stl_dir, "shape*.stl")):
        m = o3d.io.read_triangle_mesh(stl_file)
        composite_mesh += m
        surfaces.append(m)

    center = composite_mesh.get_center()
    composite_mesh.translate(-center)
    composite_mesh.scale(0.001, center=[0.,0.,0.])
    print(f"loaded {len(surfaces)} stls")

    seams = []
    for seam_filename in glob.glob(os.path.join(seam_dir, "expected_feature_*_segment*.ply")):
        s = o3d.io.read_point_cloud(seam_filename)
        seams.append(s)
    print(f"loaded {len(seams)} seams")

    o3d.visualization.draw_geometries([composite_mesh]+seams)


show_seams_on_stl("../data/results/Yoshi_Hex", "../data/yoshi_hex/")