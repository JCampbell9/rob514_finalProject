#!/usr/bin/env python


import rospy
import sys
import csv
import glob
import sys
import numpy as np
import os

from kinova_scripts.srv import Joint_angles



if __name__ == '__main__':
    ##########################################################################################
    # Reads the joint angles and sends them to the plannar to create
    ##########################################################################################

    try:  # get input from command line for which file to use
	    n = sys.argv[1]
    except:
	    n = 6


    rospy.init_node('joint_angles', argv=sys.argv, disable_signals=True)

    directory = os.path.dirname(os.path.realpath(__file__))

    angles = np.zeros((1,13))

    # set the file number parameter so other programs update
    rospy.set_param('file_number', str(n))

    # get angles
    with open(directory + '/final_test/test_data/Matrices/Angles_Optimized.2.csv') as f:  # '/final_test/test_data/Matrices/Angles_' + str(n) +'.0.csv'
    # with open(directory + '/final_test/test_data/Matrices/Angles_' + str(n) +'.0.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                angles[j][i] = float(col)

    arm_angles = angles[0]
    print(arm_angles)
    rospy.wait_for_service('joint_angles')

    set_angles = rospy.ServiceProxy('joint_angles', Joint_angles)

    # send the angles to the plannar to move the arm
    try:
        answer = set_angles(arm_angles)
    except rospy.ServiceException as e:
        rospy.logwarn('Service call failed for: {0}'.format(e))

    rospy.loginfo( '{0}'.format(answer.success))

    rospy.signal_shutdown('{0}'.format(answer.success))