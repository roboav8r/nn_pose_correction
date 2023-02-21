#!/usr/bin/env python

from __future__ import print_function

#from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy
import tf
import ros_numpy
import numpy as np
from sklearn import neighbors
from nn_pose_correction.srv import PoseCorrection
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker

def handle_nn_pose_corr(req):
    print("Server heard %s "%(req))

    # Get pointcloud message
    depth_msg = rospy.wait_for_message('/camera/depth/points', PointCloud2)
    # print("Server heard pointcloud msg %s "%(depth_msg))

    # Initialize markers for visualization
    markerArray = MarkerArray()
    init_marker = Marker()
    init_marker.header.frame_id = depth_msg.header.frame_id
    init_marker.type = init_marker.SPHERE
    init_marker.scale.x = 0.02
    init_marker.scale.y = 0.02
    init_marker.scale.z = 0.02
    init_marker.color.a = 1.0
    init_marker.color.r = 1.0
    init_marker.color.g = 0.0
    init_marker.color.b = 0.0
    init_marker.pose.orientation.w = 1.0
    corr_marker = Marker()
    corr_marker.header.frame_id = req.initial_pose.header.frame_id
    corr_marker.type = corr_marker.SPHERE
    corr_marker.scale.x = 0.02
    corr_marker.scale.y = 0.02
    corr_marker.scale.z = 0.02
    corr_marker.color.a = 1.0
    corr_marker.color.r = 0.0
    corr_marker.color.g = 1.0
    corr_marker.color.b = 0.0
    corr_marker.pose.orientation.w = 1.0

    # Transform target point into depth message frame
    t = rospy.Time.now()
    listener.waitForTransform(depth_msg.header.frame_id,req.initial_pose.header.frame_id,t,rospy.Duration(5))
    converted_pose = listener.transformPose(depth_msg.header.frame_id, req.initial_pose)

    # Publish initial pose marker
    init_marker.pose.position = converted_pose.pose.position
    markerArray.markers.append(init_marker)
    marker_pub.publish(markerArray)

    # Format data as numpy arrays
    pcl_arr = ros_numpy.numpify(depth_msg)
    pcl_xyz = ros_numpy.point_cloud2.get_xyz_points(pcl_arr)
    print("Server computed xyz array %s "%(str(np.shape(pcl_xyz))))

    # tgt_arr = ros_numpy.numpify(req.initial_pose)
    tgt_xyz = np.array([[converted_pose.pose.position.x, converted_pose.pose.position.y, converted_pose.pose.position.z ]])
    print("Target xyz (pointcloud frame) %s "%(tgt_xyz))

    # TODO Remove points outside a given radius?

    # Run inference, get corrected pose in pointcloud frame
    n_neighbors = 30
    knn = neighbors.KNeighborsRegressor(n_neighbors, weights="uniform")
    corr_xyz = knn.fit(pcl_xyz,pcl_xyz).predict(tgt_xyz)
    print("Corrected xyz (pointcloud frame) %s "%(corr_xyz))

    # Populate corrected pose message and publish
    corrected_pose = converted_pose
    corrected_pose.pose.position.x = corr_xyz[0,0]
    corrected_pose.pose.position.y = corr_xyz[0,1]
    corrected_pose.pose.position.z = corr_xyz[0,2]

    # Transform corrected pose from pointcloud into original frame
    corrected_pose_original = listener.transformPose(req.initial_pose.header.frame_id, converted_pose)

    # Visualize
    corr_marker.pose.position = corrected_pose_original.pose.position
    markerArray.markers.append(init_marker)
    markerArray.markers.append(corr_marker)
    marker_pub.publish(markerArray)

    return corrected_pose_original



if __name__ == "__main__":
    rospy.init_node('nn_pose_corr_server')

    s = rospy.Service('nn_pose_corr', PoseCorrection, handle_nn_pose_corr)
    marker_pub = rospy.Publisher('nn_pose_markers', MarkerArray)

    listener = tf.TransformListener()

    print("Ready to correct pose using nearest-neighbor regression.")
    rospy.spin()