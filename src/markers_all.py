#! /usr/bin/env python


import rospy

import tf
from tf.transformations import quaternion_from_euler
import os
import numpy as np
import csv
import glob
import sys

from visualization_msgs.msg import Marker


class Markers:
    """"""

    def __init__(self):

        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        # self.arm_pub = rospy.Publisher('arm_markers', Marker, queue_size=10)
        self.camera_pub = rospy.Publisher('camera_markers', Marker, queue_size=10)

        # self.arm_marker()
        self.marker_dict = {}
        self.marker_names = []
        self.markers = []

        self.file_number = rospy.get_param('file_number')

        self.camera_marker()

        rate = rospy.Rate(10)

        # Publish the marker at 10Hz.
        while not rospy.is_shutdown():
            file_num = rospy.get_param('file_number')
            if file_num != self.file_number:
                self.file_number = file_num
                self.camera_marker()

            for i in range(len(self.markers)):
                self.camera_pub.publish(self.markers[i])

            rate.sleep()




    def camera_marker(self):
        self.readfile()
        self.marker_setup()
        

    def arm_marker(self):

        self.readfile()
        

    def marker_setup(self):
        
        self.markers[:] = self.marker_names[:]
        for i in range(len(self.markers)):

            # rospy.logerr(self.marker_dict[self.marker_names[i]])
            self.markers[i] = Marker()
            self.markers[i].header.frame_id = '/world_frame'
            self.markers[i].type = self.markers[i].CUBE
            self.markers[i].id = i
            self.markers[i].action = self.markers[i].ADD
            self.markers[i].scale.x = 0.036
            self.markers[i].scale.y = 0.036
            self.markers[i].scale.z = 0.01

            self.markers[i].color.r = 0
            self.markers[i].color.g = 0
            self.markers[i].color.b = 1
            self.markers[i].color.a = 1

            pose = self.marker_dict[self.marker_names[i]]

            self.markers[i].pose.position.x = pose[0]
            self.markers[i].pose.position.y = pose[1]
            self.markers[i].pose.position.z = pose[2]
            
            quan = quaternion_from_euler(np.pi * pose[3] / 180, np.pi * pose[4] / 180, np.pi * pose[5] / 180)
            self.markers[i].pose.orientation.x = quan[0]
            self.markers[i].pose.orientation.y = quan[1]
            self.markers[i].pose.orientation.z = quan[2]
            self.markers[i].pose.orientation.w = quan[3]

    def readfile(self):

        with open(self.dir_path + '/final_test/data_file_' + self.file_number + '.csv') as f:
            reader = csv.reader(f)
            next(reader)
            for j, row in enumerate(reader):
                data_list = []
                for i in range(1, len(row)):
                    data_list.append(float(row[i]))

                self.marker_names.append(str(row[0]))
                self.marker_dict[str(row[0])] = data_list








if __name__ == '__main__':

    rospy.init_node('camera_markers')

    marker = Markers()

    rospy.spin()