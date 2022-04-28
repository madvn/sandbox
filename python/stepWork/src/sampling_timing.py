import time
import tqdm
import json

import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d


def time_uniform_sampling(mesh, num_points_list, num_trials):
    times = []

    for num_points in num_points_list:
        this_times = []
        for trial in tqdm.tqdm(np.arange(num_trials), desc=str(num_points)):
            tic = time.time()
            pts = mesh.sample_points_uniformly(int(num_points))
            toc = time.time()
            this_times.append(toc-tic)
        times.append(this_times)

    return times


def time_poisson_sampling(mesh, num_points_list, num_trials):
    times = []

    for num_points in num_points_list:
        this_times = []
        for trial in tqdm.tqdm(np.arange(num_trials), desc=str(num_points)):
            tic = time.time()
            pts = mesh.sample_points_poisson_disk(int(num_points))
            toc = time.time()
            this_times.append(toc-tic)
        times.append(this_times)

    return times


# def plotit(uniform_times, poisson_times, num_points_list, name):
#     fig_tiles = np.ceil(np.sqrt(len(num_points_list)))
#     plt.figure(figsize=[8,8])
#     for i, num_points in enumerate(num_points_list):
#         plt.subplot(fig_tiles, fig_tiles, i+1)
#         plt.hist(uniform_times[i], alpha=0.5)
#         plt.hist(poisson_times[i], alpha=0.5)
#         plt.title("Num points = {}".format(num_points))
#     plt.suptitle(name)
#     plt.tight_layout()
#     plt.savefig("../data/results/{}_times.png".format(name))
#
#     d = {"uniform_times": uniform_times, "poisson_times":poisson_times}
#     with open("../data/results/{}_times.json".format(name), "w") as f:
#         json.dump(d, f, indent=4)


def plotit(uniform_times, num_points_list, name):
    fig_tiles = np.ceil(np.sqrt(len(num_points_list)))
    bins = np.linspace(0,1.1*np.max(uniform_times), 100)
    plt.figure(figsize=[4,3])
    for i, num_points in enumerate(num_points_list):
        plt.hist(uniform_times[i], bins=bins, alpha=0.5, label="{:.0e} pts".format(num_points))
    plt.title("Mesh: {} | Num trials: {}".format(name, len(uniform_times[0])))
    plt.legend()
    plt.xlabel("Time")
    plt.tight_layout()
    plt.savefig("../data/results/{}_times.png".format(name))

    d = {"uniform_times": uniform_times}
    with open("../data/results/{}_times.json".format(name), "w") as f:
        json.dump(d, f, indent=4)


#############
# config
#############
num_points_list = [1e3, 1e4, 1e5, 1e6, 5e6, 10e6]
num_trials = 1000

# box
print("\nBox")
m = o3d.geometry.TriangleMesh.create_box(0.1,0.1,0.1)
uniform_times = time_uniform_sampling(m, num_points_list, num_trials)
plotit(uniform_times, num_points_list, "Box")

# cylinder
print("\nCylinder")
m = o3d.geometry.TriangleMesh.create_cylinder()
uniform_times = time_uniform_sampling(m, num_points_list, num_trials)
plotit(uniform_times, num_points_list, "Cylinder")

# box+cylinder
print("\nBox+Cylinder")
m = o3d.geometry.TriangleMesh.create_box(0.1,0.1,0.1) + o3d.geometry.TriangleMesh.create_cylinder(radius=0.05, height=0.2)
uniform_times = time_uniform_sampling(m, num_points_list, num_trials)
plotit(uniform_times, num_points_list, "Box+Cylinder")

# yoshi
print("\nYoshi part")
m = o3d.io.read_triangle_mesh("../data/yoshi_hex/ideal_mesh.ply")
uniform_times = time_uniform_sampling(m, num_points_list, num_trials)
plotit(uniform_times, num_points_list, "Yoshi part")

# Cincy fan
print("\nCincy wheel")
m = o3d.io.read_triangle_mesh("../data/cincyFan/56260-13 (RBE-13 Wheel).PLY")
uniform_times = time_uniform_sampling(m, num_points_list, num_trials)
plotit(uniform_times, num_points_list, "Cincy wheel")



