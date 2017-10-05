#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

from segmentation_controller import *


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_msg):

# Exercise-2 TODOs:
    detected_objects_labels = []
    detected_objects = []
    
    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(ros_msg)
   
    # Voxel Grid Downsampling
    cloud_filtered = voxel_downsampling(pcl_data)

    # PassThrough Filter
    cloud_filtered = passthrough_filter(cloud_filtered)

    # RANSAC Plane Segmentation
    inliers, coefficients = ransac_plane_segmentation(cloud_filtered)
  
    # Extract inliers and outliers
    pcl_objects = cloud_filtered.extract(inliers, negative=True)
    pcl_table = cloud_filtered.extract(inliers, negative=False)

    ##
    pcl_objects = statistical_outlier_filter(pcl_objects)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(pcl_objects)
    cluster_indices = euclidean_clustering(white_cloud)

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud = color_clusters(white_cloud, cluster_indices)

    # Convert PCL data to ROS messages
    ros_objects = pcl_to_ros(pcl_objects)
    ros_table = pcl_to_ros(pcl_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # Publish ROS messages
    pcl_objects_pub.publish(ros_objects)
    pcl_table_pub.publish(ros_table)
    pcl_cluster_cloud.publish(ros_cluster_cloud)
    

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = pcl_objects.extract(pts_list)

        # Compute the associated feature vector
        ros_cluster = pcl_to_ros(pcl_cluster)
        color_hist = compute_color_histograms(ros_cluster)
        normals = get_normals(ros_cluster)
        normal_hist = compute_normal_histograms(normals)
        #print(color_hist.shape, normal_hist.shape)
        feature = np.append(color_hist, normal_hist)

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)
    

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):
    # TODO: Initialize variables
    object_list_param = rospy.get_param('/object_list')    
    dropbox = rospy.get_param('/dropbox')
    labels = []
    centroids = []
    dict_list = []

    # TODO: Get/Read parameters
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for idx, object in enumerate(object_list_param):

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        foundIdx = labels.index(object['name'])
        centroid = centroids[foundIdx]
        print('sup there')
        print(object['name'], centroid)
        

        # TODO: Create 'place_pose' for the object


        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        print('hi')
        rospy.wait_for_service('pick_place_routine')
        print('end')
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            

            ####
            test_scene_number = Int32()
            test_scene_number.data = 1
            object_name = String()
            object_name.data = object['name']
            which_arm = String()
            if object['group'] == 'green':                                                    
                which_arm.data = 'left'
            else:
                which_arm.data = 'right'
            pick_pose = Pose()
    
            pick_pose.position.x = centroid[0]
            pick_pose.position.y = centroid[1]
            pick_pose.position.z = centroid[2]

            pick_pose.orientation.x = 0.0
            pick_pose.orientation.y = 0.0
            pick_pose.orientation.z = 0.0
            pick_pose.orientation.w = 0.0
            place_pose = Pose()
            place_pose.position.x = dropbox[0]['position'][0]
            place_pose.position.y = dropbox[0]['position'][1]
            place_pose.position.z = dropbox[0]['position'][2]
            place_pose.orientation.x = 0.0
            place_pose.orientation.y = 0.0
            place_pose.orientation.z = 0.0
            place_pose.orientation.w = 0.0
            print("making dict")
            yaml_dict = make_yaml_dict(test_scene_number, object_name, which_arm, pick_pose, place_pose)
            dict_list.append(yaml_dict)
            print("made dict")
            #resp = pick_place_routine(test_scene_number, object_name, which_arm, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    print('sending')
    send_to_yaml('yaml_out', dict_list)
    print(dict_list)


if __name__ == '__main__':

     # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_cloud = rospy.Publisher("/pcl_cluster_cloud", PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
