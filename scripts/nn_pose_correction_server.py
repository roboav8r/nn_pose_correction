#!/usr/bin/env python

from __future__ import print_function

#from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy
import tf2_ros
import ros_numpy
import numpy as np
from sklearn import neighbors
from nn_pose_correction.srv import PoseCorrection
from sensor_msgs.msg import PointCloud2

def handle_nn_pose_corr(req):
    print("Server heard %s "%(req))

    # Get pointcloud message
    depth_msg = rospy.wait_for_message('/camera/depth/points', PointCloud2)
    # print("Server heard pointcloud msg %s "%(depth_msg))

    # TODO Transform target point into depth message frame
    converted_pose = req.initial_pose

    # Format data as numpy arrays
    pcl_arr = ros_numpy.numpify(depth_msg)
    pcl_xyz = ros_numpy.point_cloud2.get_xyz_points(pcl_arr)
    print("Server computed xyz array %s "%(str(np.shape(pcl_xyz))))

    # tgt_arr = ros_numpy.numpify(req.initial_pose)
    tgt_xyz = np.array([[converted_pose.pose.position.x, converted_pose.pose.position.y, converted_pose.pose.position.z ]])
    print("Target xyz (pointcloud frame) %s "%(tgt_xyz))

    # TODO Remove points outside a given radius?

    # Run inference, get corrected pose in pointcloud frame
    n_neighbors = 15
    knn = neighbors.KNeighborsRegressor(n_neighbors, weights="uniform")
    corr_xyz = knn.fit(pcl_xyz,pcl_xyz).predict(tgt_xyz)
    print("Corrected xyz (pointcloud frame) %s "%(corr_xyz))

    # TODO Transform corrected pose from pointcloud into original frame

    # Populate corrected pose message and publish
    corrected_pose = req.initial_pose
    corrected_pose.pose.position.x = corr_xyz[0,0]
    corrected_pose.pose.position.y = corr_xyz[0,1]
    corrected_pose.pose.position.z = corr_xyz[0,2]
    return corrected_pose


def nn_pose_corr_server():
    rospy.init_node('nn_pose_corr_server')

    s = rospy.Service('nn_pose_corr', PoseCorrection, handle_nn_pose_corr)


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print("Ready to correct pose using nearest-neighbor regression.")
    rospy.spin()

if __name__ == "__main__":
    nn_pose_corr_server()