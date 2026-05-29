import pymeshlab


def genSTL(point_cloud_file, octree_depth = 9, nearest_neighbors = 10):

    #Octree depth — controls reconstruction resolution:
    #6-7  : Coarse         | Quick preview
    #8-9  : Medium         | Smooth organic shapes
    #10-11: Fine           | Sharp-edged shapes like cuboids
    #12+  : Very fine      | High-detail models, slow

    # Nearest neighbours — controls normal estimation quality:
    #   3-5  : Very local      | Noisy normals, sensitive to outliers
    #   6-10 : Small           | Good for sharp edges and cuboid faces
    #   10-20: Moderate        | Balanced, good general purpose
    #   20-30: Large           | Smoother normals, rounded edges
    #   30+  : Very large      | Over-smoothed, loses sharp features

    load_param = {
        "rowtoskip" : 2,
        "strformat" : "X Y Z",
        "separator" : "SPACE",
        "onerror" : "skip"
    }

    meshSet = pymeshlab.MeshSet();

    meshSet.load_new_mesh(point_cloud_file, **load_param)
    meshSet.compute_normal_for_point_clouds(k = nearest_neighbors)
    meshSet.generate_surface_reconstruction_screened_poisson(preclean = True, depth = octree_depth)
    meshSet.save_current_mesh(point_cloud_file.replace(".txt", ".stl"))
