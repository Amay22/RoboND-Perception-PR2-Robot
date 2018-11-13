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

scene_num = 3 # the test scene used in the model

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

def outlier_filter(point_cloud, neighboring_pts, scale_factor):
    ''' Removes point_cloud data outliers based on neighboring_pts
    and standard deviation on the scale_factor
    :param point_cloud: pcl filtered clusters
    :param neighboring_pts: number of neighbouring points to be analyzed
    :param scale_factor: threshold of the standard deviation to be analyzed
    :returns: filtered point cloud without the outliers
    '''
    outlier_filter = point_cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(neighboring_pts)
    outlier_filter.set_std_dev_mul_thresh(scale_factor)
    return outlier_filter.filter()

def voxel_grid_filter(point_cloud, leaf_size):
    ''' Voxel grid downsampling the point_cloud data
    :param point_cloud: pcl filtered clusters
    :param leaf_size: the measurement size of each unit
    :returns: point_cloud Voxel grid downsampled
    '''
    voxel_grid = point_cloud.make_voxel_grid_filter()
    voxel_grid.set_leaf_size(leaf_size, leaf_size, leaf_size)
    return voxel_grid.filter()

def passthrough_filter(point_cloud, axis, axis_min, axis_max):
    ''' Filters the point cloud to only the data within the min and max values
    :param point_cloud: pcl filtered clusters
    :param axis: axis to be filter along.
                 x is in the direction of the camera,
                 y is the left & right of the camera,
                 z up & down
    :param axis_min: min threshold distance for filter
    :param axis_min: max threshold distance for filter
	:returns: passthrough filtered point_cloud
	'''
    passthrough = point_cloud.make_passthrough_filter()
    passthrough.set_filter_field_name(axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

def ransac_filter(point_cloud, max_distance):
    ''' RANSAC plane segmentation for segmenting the table
    :param point_cloud: pcl filtered clusters
    :param max_distance: max length of a point to be considered to fit the model
    :returns: inlier indices and model coefficients of the segment
    '''
    segmenter = point_cloud.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_distance_threshold(max_distance)
    return segmenter.segment()

def euclidean_cluster(white_cloud):
    ''' Euclidean Clustering
    :param white_cloud: cloud points containing only xyz data
    :returns: Euclidean cluster locations of the white_cloud
    '''
    euclidean = white_cloud.make_EuclideanClusterExtraction()
    euclidean.set_ClusterTolerance(0.02)
    euclidean.set_MinClusterSize(50)
    euclidean.set_MaxClusterSize(25000)
    euclidean.set_SearchMethod(white_cloud.make_kdtree())
    return euclidean.Extract()

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    pcl_filtered = ros_to_pcl(pcl_msg)
    # Statistical Outlier Filtering
    pcl_filtered = outlier_filter(pcl_filtered, neighboring_pts=5, scale_factor=0.05)
    # Voxel Grid Downsampling
    pcl_filtered = voxel_grid_filter(pcl_filtered, 0.005)
    # PassThrough Filter
    pcl_filtered = passthrough_filter(pcl_filtered, axis='x', axis_min=0.1, axis_max=0.9)
    pcl_filtered = passthrough_filter(pcl_filtered, axis='y', axis_min=-0.47, axis_max=0.47)
    pcl_filtered = passthrough_filter(pcl_filtered, axis='z', axis_min=0.6, axis_max=0.9)
    # RANSAC Plane Segmentation
    inliers, coefficients = ransac_filter(pcl_filtered, max_distance=0.01)
    # Extract inliers and outliers
    cloud_table = pcl_filtered.extract(inliers, negative=False)
    cloud_objects = pcl_filtered.extract(inliers, negative=True)
    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    cluster_indices = euclidean_cluster(white_cloud)
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for i, indexes in enumerate(cluster_indices):
        for j, idx in enumerate(indexes):
            color_cluster_point_list.append([white_cloud[idx][0],
                                             white_cloud[idx][1],
                                             white_cloud[idx][2],
                                             rgb_to_float(cluster_color[i])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # Exercise-3 TODOs:
    detected_objects_labels = []
    detected_objects = []
    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):
        ## Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True, nbins=44)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals, nbins=20)
        feature = np.concatenate((chists, nhists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4
        object_markers_pub.publish(make_label(label, label_pos, index))
        # Add the detected object to the list of detected objects.
        detected_object = DetectedObject()
        detected_object.label = label
        detected_object.cloud = ros_cluster
        detected_objects.append(detected_object)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
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

	# Initialize variables
    dict_list = []
    labels = []
    centroids = []
    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    # Parse parameters into individual variables
    for i in range(len(object_list_param)):
        object_name = object_list_param[i]['name']
        object_group = object_list_param[i]['group']

        # Loop through the pick list
        for j, obj in enumerate(object_list):
            labels.append(obj.label)
            points_arr = ros_to_pcl(obj.cloud).to_array()
            centroids.append(np.mean(points_arr, axis=0)[:3])

            # check if the object is the next object on the list to relocate
            if obj.label == object_name:
                labels.append(obj.label)
                points_arr = ros_to_pcl(obj.cloud).to_array()
                # Get the PointCloud for a given object and obtain it's centroid
                centroids.append(np.mean(points_arr, axis=0)[:3])
                pick_pose = Pose()
                pick_pose.position.x = np.asscalar(centroids[j][0])
                pick_pose.position.y = np.asscalar(centroids[j][1])
                pick_pose.position.z = np.asscalar(centroids[j][2])
                # Create 'place_pose' for the object
                dropbox_param = rospy.get_param('/dropbox')
                place_pose = Pose()
                arm_name = String()
                if object_group == dropbox_param[0]['group']:
                    # Assign the left arm to be used for pick_place
                    arm_name.data = dropbox_param[0]['name']
                    place_pose.position.x = dropbox_param[0]['position'][0]-0.03 - 0.05 * i
                    place_pose.position.y = dropbox_param[0]['position'][1] + 0.03 * i
                    place_pose.position.z = dropbox_param[0]['position'][2]
                elif object_group == dropbox_param[1]['group']:
                    # Assign the right arm to be used for pick_place
                    arm_name.data = dropbox_param[1]['name']
                    place_pose.position.x = dropbox_param[1]['position'][0]-0.03 - 0.05 * i
                    place_pose.position.y = dropbox_param[1]['position'][1] - 0.06 * i
                    place_pose.position.z = dropbox_param[1]['position'][2]

	            # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                test_scene_num = Int32()
                test_scene_num.data = scene_num
                obj_name = String()
                obj_name.data = object_name
                yaml_dict = {}
                yaml_dict = make_yaml_dict(test_scene_num, arm_name, obj_name, pick_pose, place_pose)
                dict_list.append(yaml_dict)

    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        # Insert your message variables to be sent as a service request
        resp = pick_place_routine(test_scene_num, obj_name, arm_name, pick_pose, place_pose)

        print ("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    # Output the request parameters into the enviroment output yaml file
    send_to_yaml('output_' + str(scene_num) + '.yaml', dict_list)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", PointCloud2, pcl_callback, queue_size=1)
    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []
    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
