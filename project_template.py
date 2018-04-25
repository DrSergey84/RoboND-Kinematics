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
def pcl_callback(pcl_msg):

    cloud = ros_to_pcl(pcl_msg)
    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.005
    #pcl.save(cloud, 'cloud_objects.pcd')
    #print('file saved')

    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud.make_voxel_grid_filter()

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # PassThrough filter
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()
    passthrough = cloud_filtered.make_passthrough_filter()

    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.4
    axis_max = 1.0
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()
 
    # RANSAC plane segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    #Extract inliers ( table )
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    # Extract outliers ( objects )
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloud_objects.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(10)

    # Set threshold scale factor
    x = 0.2

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_objects = outlier_filter.filter()

    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(40)
    ec.set_MaxClusterSize(2500)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    #Assign a color corresponding to each segmented object in scene
    #cluster_color = get_color_list(len(cluster_indices))

    #color_cluster_point_list = []

    #for j, indices in enumerate(cluster_indices):
    #   for i, indice in enumerate(indices):
    #      color_cluster_point_list.append([white_cloud[indice][0],
    #                                       white_cloud[indice][1],
    #                                       white_cloud[indice][2],
    #                                       rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    #cluster_cloud = pcl.PointCloud_PointXYZRGB()
    #cluster_cloud.from_list(color_cluster_point_list)
    #pcl.save(cluster_cloud, 'cloud_objects.pcd')
    #print('file saved')


    #Assign a color corresponding to each segmented object in scene
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        #labeled_features.append([feature, model_name])

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
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

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)


    # Publish the list of detected objects

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):

    # Initialize variables
    test_scene_num = Int32()
    yaml_params = []
    dict_centroid = {} # label -> centroid x,y,z tuple
    dict_arm_pos = {}  # color -> (arm, position)

    #  Object list from the server defining the order of pick and place task
    object_list_param = rospy.get_param('/object_list')
    
    # Dropbox parameters ( position of the drop boxes )
    dropbox_list_param = rospy.get_param('/dropbox')
    red_db_position = dropbox_list_param[0]['position']
    green_db_position = dropbox_list_param[1]['position']

    # Which test scene is this?
    scene_num = 3
    test_scene_num.data = scene_num

    # Generate dictionaries to retrieve drop box position, centroid value, arm to use later
    for object in detected_objects:
      points_arr = ros_to_pcl(object.cloud).to_array()
      dict_centroid[object.label] = np.mean(points_arr, axis=0)[:3]

    dict_arm_pos['green'] = ('right', green_db_position)
    dict_arm_pos['red'] =   ('left' , red_db_position)


    #Now populate the yamp parameters
    for d in object_list_param:
       # Initialize a variable
       object_name = String()
       object_name.data = d['name']
       # Create 'arm_name' message 
       arm_name = String()
       arm_name.data = dict_arm_pos[d['group']][0]
       # Setup pick pose
       pick_pose = Pose()

       # If predictor has managed to detect this object
       if ( d['name'] in dict_centroid):
           centroid = dict_centroid[d['name']]
           pick_pose.position.x = np.asscalar(centroid[0])
           pick_pose.position.y = np.asscalar(centroid[1])
           pick_pose.position.z = np.asscalar(centroid[2])
           # Setup place pose
           place_pose = Pose()
           dropbox_position = dict_arm_pos[d['group']][1]
           place_pose.position.x = dropbox_position[0]
           place_pose.position.y = dropbox_position[1]
           place_pose.position.z = dropbox_position[2]
           yaml_params.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))


    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    try:
         pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

         # TODO: Insert your message variables to be sent as a service request
         #resp = pick_place_routine(scene_num, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

         #print ("Response: ",resp.success)

    except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    file_name = "output_{}.yaml".format(scene_num)
    send_to_yaml(file_name, yaml_params)


if __name__ == '__main__':

    rospy.init_node('clustering', anonymous=True)
    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model3.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    #Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
