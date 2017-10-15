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

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()    
    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.01
    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
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

    # TODO: RANSAC Plane Segmentation
    # TODO: RANSAC Plane Segmentation
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

    # TODO: Extract inliers and outliers
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    # TODO: Euclidean Clustering
    #Euclidean Clustering
    #In order to perform Euclidean Clustering, you must first 
    #construct a k-d tree from the cloud_objects point cloud.
    #The k-d tree data structure is used in the Euclidian Clustering algorithm 
    #to decrease the computational burden of searching for neighboring points. 
    #While other efficient algorithms/data structures for nearest neighbor 
    #search exist, PCL's Euclidian Clustering algorithm only supports k-d trees.

    #To construct a k-d tree, you first need to convert your XYZRGB point cloud to 
    #XYZ, because PCL's Euclidean Clustering algorithm requires a point cloud with only 
    #spatial information. To create this colorless cloud, (which I'll call white_cloud), 
    #search again in pcl_helper.py to find the function you need to convert XYZRGB to XYZ. 
    #Next, construct a k-d tree from it. To accomplish this, add the following code 
    #to the pcl_callback() function in your node:

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    #Once your k-d tree has been constructed, you can perform the cluster extraction like this:
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(1550)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Visualization
    #Cluster Visualization
    #Now that we have lists of points for each object (cluster_indices), you can perform 
    #the final step of this exercise, visualizing the results in RViz!
    #Choosing a unique color for each segmented Object
    #In order to visualize the results in RViz, you need to create one final point cloud,
    #let's call it "cluster_cloud" of type PointCloud_PointXYZRGB. 
    #This cloud will contain points for each of the segmented objects, with each set of points having a unique color.
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    #Publish ROS messages from your 
    #pcl_callback(). For now you'll just be publishing 
    #the original pointcloud itself, but later you'll change 
    #these to be the point clouds associated with the table 
    #and the objects.

    # TODO: Publish ROS msg
    object_markers_pub.publish(ros_cloud_objects)
    #pcl_table_pub.publish(ros_cloud_table)
    detected_objects_pub.publish(ros_cluster_cloud)


    # Classify the clusters!
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        sample_cloud_arr = pcl_cluster.to_array()
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cloud_objects = pcl_to_ros(pcl_cluster)
        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        # Check for invalid clouds.
        if sample_cloud_arr.shape[0] == 0:
            print('Invalid cloud detected')
        else:
            # Extract histogram features
            chists = compute_color_histograms(sample_cloud_arr, using_hsv=True)
            normals = get_normals(sample_cloud_arr)
            nhists = compute_normal_histograms(normals)
            feature = np.concatenate((chists, nhists))
            labeled_features.append([feature, model_name])
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

if __name__ == '__main__':

    # TODO: ROS node initialization
    #Initialize your ROS node. 
    #In this step you are initializing a 
    #new node called "clustering".
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    #Create Subscribers. Here we're subscribing our 
    #node to the "sensor_stick/point_cloud" topic. 
    #Anytime a message arrives, the message data 
    #(a point cloud!) will be passed to the pcl_callback() function for processing.
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    #Create Publishers. Here you're creating two new publishers to publish the point cloud data for the table and the objects on the table to topics called pcl_table and pcl_objects, respectively.

    # TODO: Create Publishers
    # Create Publishers
    # TODO: here you need to create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    #pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    #Spin while node is not shutdown. Here you're 
    #preventing your node from exiting until an intentional shutdown is invoked.
    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()