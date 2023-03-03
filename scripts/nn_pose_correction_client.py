#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from nn_pose_correction.srv import *
from geometry_msgs.msg import PoseStamped

def nn_pose_corr_client(initial_pose):
    rospy.wait_for_service('mega/grasp_refinement_john')
    try:
        nn_pose_corr = rospy.ServiceProxy('mega/grasp_refinement_john', grasp_refinement_john)
        corrected_pose = nn_pose_corr(initial_pose)
        return corrected_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":

    init_pose = PoseStamped()
    init_pose.header.frame_id = 'tripod_base_link'
    init_pose.pose.position.x = .25
    init_pose.pose.position.y = 0
    init_pose.pose.position.z = .10
    init_pose.pose.orientation.x = 0
    init_pose.pose.orientation.y = 0
    init_pose.pose.orientation.z = 0
    init_pose.pose.orientation.w = 1      

    print("Sent initial pose %s, got corrected pose %s"%(init_pose, nn_pose_corr_client(init_pose)))