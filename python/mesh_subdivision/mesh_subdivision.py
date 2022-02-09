import numpy as np
import open3d as o3d


def subdivide_mesh(mesh, threshold=0.3):
    repeat = True
    while repeat:
        repeat = False
        vertices = np.array(mesh.vertices)
        triangles = np.array(mesh.triangles)
        num_vertices = len(vertices)
        num_triangles = len(triangles)

        # subdivide mesh based on edge lengths
        new_vertices = []
        new_triangles = []
        triangle_remove_inds = []
        for i, (t1, t2, t3) in enumerate(triangles):
            v1, v2, v3 = vertices[t1], vertices[t2], vertices[t3]
            e1 = np.linalg.norm(v1 - v2)
            e2 = np.linalg.norm(v2 - v3)
            e3 = np.linalg.norm(v1 - v3)

            # Is at least one edge longer than threshold?
            if np.max([e1, e2, e3]) > threshold:
                # this triangle will get split to remove it from the mesh
                triangle_remove_inds.append(i)

                # repeat process even if one edge was split to create a conformal mesh
                repeat = True

                # only split the longest edge
                # NOT all edges that are greater than the threshold
                longest_edge_ind = np.argmax([e1, e2, e3])

                # create new vertex and 2 triangles
                # these will be added after this loop to the mesh
                #     just so we don't grow "triangles" inside this loop
                if longest_edge_ind == 0:
                    new_vertex = (v1 + v2) / 2
                    new_triangle1 = [t1, num_vertices + len(new_vertices), t3]
                    new_triangle2 = [num_vertices + len(new_vertices), t2, t3]
                elif longest_edge_ind == 1:
                    new_vertex = (v2 + v3) / 2
                    new_triangle1 = [t1, num_vertices + len(new_vertices), t3]
                    new_triangle2 = [t1, t2, num_vertices + len(new_vertices)]
                else:
                    new_vertex = (v1 + v3) / 2
                    new_triangle1 = [t1, t2, num_vertices + len(new_vertices)]
                    new_triangle2 = [num_vertices + len(new_vertices), t2, t3]
                new_vertices.append(new_vertex)
                new_triangles.append(new_triangle1)
                new_triangles.append(new_triangle2)

        # add all new vertices and triangles to mesh
        # we now have a new version of the mesh to repeat the loop with
        mesh.remove_triangles_by_index(triangle_remove_inds)
        vertices = list(vertices) + list(new_vertices)
        triangles = list(np.array(mesh.triangles)) + list(new_triangles)
        # triangles = list(triangles) + list(new_triangles)
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_degenerate_triangles()

    return mesh

cube = o3d.geometry.TriangleMesh.create_box(width=1.0, height=5.0, depth=1.0)
import copy
temp = subdivide_mesh(copy.deepcopy(cube))
o3d.visualization.draw_geometries([cube])
o3d.visualization.draw_geometries([temp])

