#! /usr/bin/env python


import rospy

import tf
import os
import numpy as np
import csv
import glob
import sys




if __name__ == '__main__':
    rospy.init_node('camera_frame_tf')

    directory = os.path.dirname(os.path.realpath(__file__))

    camera_mat = np.zeros((4, 4))
    translation_mat = np.zeros((4, 4))
    rotation_mat = np.zeros((4, 4))

    with open(directory + '/final_test/TranslationMatrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                translation_mat[j][i] = float(col)


    with open(directory + '/final_test/RotationMatrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                rotation_mat[j][i] = float(col)


    with open(directory + '/final_test/World_to_Camera_Transform_Matrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                camera_mat[j][i] = float(col)

    camera_transform = np.dot(camera_mat, np.dot(translation_mat, rotation_mat))
    camera_transform = np.linalg.inv(camera_transform)
    transform = np.dot(rotation_mat, translation_mat)
    transform = np.linalg.inv(transform)
    trans = tf.transformations.translation_from_matrix(transform)
    # trans = tf.transformations.translation_from_matrix(np.linalg.inv(translation_mat))
    # print(trans)
    rot = tf.transformations.quaternion_from_matrix(transform)
    # rot = tf.transformations.quaternion_from_matrix(np.linalg.inv(rotation_mat))
    # print(rot)
    
    trans2 = tf.transformations.translation_from_matrix(camera_transform)
    rot2 = tf.transformations.quaternion_from_matrix(camera_transform)
    
    br = tf.TransformBroadcaster()
    camera_world = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        br.sendTransform(tuple(trans), tuple(rot), rospy.Time.now(), 'world_frame', 'j2s7s300_link_base') # j2s7s300_end_effector
        camera_world.sendTransform(tuple(trans2), tuple(rot2), rospy.Time.now(), 'camera_frame', 'j2s7s300_link_base')
        rate.sleep()


