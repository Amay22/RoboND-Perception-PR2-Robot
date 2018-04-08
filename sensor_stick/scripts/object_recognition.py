#!/usr/bin/env python

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

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    filtered_pcl = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    voxel_grid_filter = filtered_pcl.make_voxel_grid_filter()

    # TODO: PassThrough Filter
    # Set the voxel (leaf) size
    voxel_grid_filter.set_leaf_size(0.01, 0.01, 0.01)
    filtered_pcl = voxel_grid_filter.filter()
    # Call the filter function to obtain the resultant downsampled point cloud
    passthrough_filter = filtered_pcl.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    passthrough_filter.set_filter_field_name('z')
    passthrough_filter.set_filter_limits(0.757, 1.1)
    filtered_pcl = passthrough_filter.filter()

    # TODO: RANSAC Plane Segmentation
    segmentater = filtered_pcl.make_segmenter()
    segmentater.set_model_type(pcl.SACMODEL_PLANE)
    segmentater.set_method_type(pcl.SAC_RANSAC)
    segmentater.set_distance_threshold(0.01)
    
    # TODO: Extract inliers and outliers
    inliers, coefficients = segmentater.segment()
    cloud_table = filtered_pcl.extract(inliers, negative=False)
    cloud_objects = filtered_pcl.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    euclidean_cluster = white_cloud.make_EuclideanClusterExtraction()
    euclidean_cluster.set_ClusterTolerance(0.02)
    euclidean_cluster.set_MinClusterSize(100)
    euclidean_cluster.set_MaxClusterSize(25000)
    euclidean_cluster.set_SearchMethod(white_cloud.make_kdtree())
    cluster_indices = euclidean_cluster.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for i, indices in enumerate(cluster_indices):
        for j, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[i])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs: 
    detected_objects_labels = []
    detected_objects = []
    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, points_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(points_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        color_hists = compute_color_histograms(ros_cluster, using_hsv=True, nbins=44)
        normal_hists = compute_normal_histograms(get_normals(ros_cluster), nbins=20)
        feature = np.concatenate((color_hists, normal_hists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[points_list[0]])
        label_pos += 0.4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        detected_object = DetectedObject()
        detected_object.label = label
        detected_object.cloud = ros_cluster
        detected_objects.append(detected_object)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    point_cloud_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('training_set.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
