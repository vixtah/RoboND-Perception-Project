from pcl_helper import *

def voxel_downsampling(pcl_data):
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE = .01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    return cloud_filtered

def passthrough_filter(pcl_data):
    passthrough = pcl_data.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = .595
    axis_max = .8
    passthrough.set_filter_limits(axis_min, axis_max)
    z_filtered = passthrough.filter()

    passthrough = z_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -.4
    axis_max = .4
    passthrough.set_filter_limits(axis_min, axis_max)

    cloud_filtered = passthrough.filter()
    return cloud_filtered

def ransac_plane_segmentation(pcl_data):
    seg = pcl_data.make_segmenter()

    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_distance = .01
    seg.set_distance_threshold(max_distance)

    inliers, coefficients = seg.segment()
    return (inliers, coefficients)

def euclidean_clustering(white_cloud):
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    
    ec.set_ClusterTolerance(0.03)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(100000)

    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    return cluster_indices

def color_clusters(white_cloud, cluster_indices):
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud

def statistical_outlier_filter(pcl_data):
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(5)
    x = 0.0001
    outlier_filter.set_std_dev_mul_thresh(x)
    pcl_data = outlier_filter.filter()    
    return pcl_data



