#! /usr/bin/env python


import rospy

import tf
import os
import numpy as np
from math import sin, cos, pi
import csv
import glob
import sys

from visualization_msgs.msg import Marker


directory = os.path.dirname(os.path.realpath(__file__))

def marker_setup():

    markers = [0, 1, 2, 3, 4, 5, 6]
    poses = [[0, 0, -0.005], [0.30, 0.25, 0], [-0.30, 0.25, 0], [-0.30, -0.25, 0], [0.30, -0.25, 0], [0, 0, 0], [0, 0, 0]]
    scales = [[0.60, 0.50, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01], [0.036, 0.036, 0.01]]
    colors = [[0, 1, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5], [1, 0, 0, .5]]
    for i in range(len(markers)):
        markers[i] = Marker()
        markers[i].header.frame_id = '/world_frame'
        markers[i].type = markers[i].CUBE
        markers[i].id = i
        markers[i].action = markers[i].ADD
        markers[i].scale.x = scales[i][0] 
        markers[i].scale.y = scales[i][1]
        markers[i].scale.z = scales[i][2]
        markers[i].color.r = colors[i][0] 
        markers[i].color.g = colors[i][1]
        markers[i].color.b = colors[i][2]
        markers[i].color.a = colors[i][3]
        markers[i].pose.position.x = poses[i][0]
        markers[i].pose.position.y = poses[i][1]
        markers[i].pose.position.z = poses[i][2]

    return markers



        
def ee_palm():
    ee_palm_mat = np.zeros((4, 4))

    with open(directory + '/final_test/EE_to_Palm_Transform_Matrix_2.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                ee_palm_mat[j][i] = float(col)

    rot_auruco_ee = tf.transformations.euler_matrix(pi/2, pi, 0)
    
    ee_palm_mat = np.dot(np.linalg.inv(rot_auruco_ee), ee_palm_mat)
    trans = tf.transformations.translation_from_matrix(ee_palm_mat)
    rot = tf.transformations.quaternion_from_matrix(ee_palm_mat)
    # np.linalg.inv(

    return trans, rot


if __name__ == '__main__':

    rospy.init_node('markers', argv=sys.argv)

    markers = marker_setup()
    
    trans, rot = ee_palm()

    markers[6].header.frame_id = 'j2s7s300_end_effector'
    markers[6].pose.position.x = trans[0] / 100
    markers[6].pose.position.y = trans[1] / 100
    markers[6].pose.position.z = trans[2] / 100
    markers[6].pose.orientation.x = rot[0]
    markers[6].pose.orientation.y = rot[1]
    markers[6].pose.orientation.z = rot[2]
    markers[6].pose.orientation.w = rot[3]

    # Set up a publisher.  We're going to publish on a topic called balloon.
    publisher = rospy.Publisher('markers', Marker, queue_size=10)

    # Set a rate.  10 Hz is a good default rate for a marker moving with the Fetch robot.
    rate = rospy.Rate(10)

    # Publish the marker at 10Hz.
    while not rospy.is_shutdown():
        for i in range(len(markers)):
            publisher.publish(markers[i])

        rate.sleep()