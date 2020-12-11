#! usr/bin/env python3

# takes in the calibrated translation and rotation matrix with the recorded transform from the arm and gives the
# location of the end effector in the world frame.

import numpy as np
import csv
import sys
import os
import pandas as pd

directory = os.path.dirname(os.path.realpath(__file__))

# sets up matrices to read the stored matrices into
tran_mat = np.zeros((4, 4))
translation_mat = np.zeros((4, 4))
rotation_mat = np.zeros((4, 4))

########################################################################################################################
# Read in the translation rotation and transformation matrices
########################################################################################################################
def get_ee_world_location():
    with open(directory + '/TranslationMatrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                translation_mat[j][i] = float(col)


    with open(directory + '/RotationMatrix.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                rotation_mat[j][i] = float(col)


    with open(directory + '/final_test/test_data/Matrices/TransformMatrix_9.0.csv') as f:
        reader = csv.reader(f)
        for j, row in enumerate(reader):
            for i, col in enumerate(row):
                tran_mat[j][i] = float(col)


    ########################################################################################################################
    # Calculate the end effector location in the world frame
    ########################################################################################################################

    loc = np.dot(rotation_mat, np.dot(translation_mat, np.dot(tran_mat, np.transpose([0, 0, 0, 1]))))
    print('The location of the end effector')
    print(loc)
    print('\n')

    # load the ArUco Markers information
    ArUco_data = pd.read_csv(directory + '/final_test/data_file_9.csv')
    palm_data = ArUco_data[ArUco_data.location == 'palm']
    palm_data = np.array(palm_data)
    Palm_in_ArUco = np.append(palm_data[0, [1, 2, 3]], [1])
    print('Palm: ArUco Marker Location')
    print(Palm_in_ArUco)
    print('\n')

    EE_to_P = pd.read_csv(directory + '/final_test/EE_to_Palm.csv', header=None)
    EE_to_P = np.array(EE_to_P)

    P_to_world = pd.read_csv(directory + '/final_test/P_to_world.csv', header=None)
    P_to_world = np.array(P_to_world)

    move = np.eye(4)
    move[:, 3] = loc.T

    Palm_in_world = np.dot(move, np.dot(P_to_world, np.dot(EE_to_P, [0, 0, 0, 1])))
    print('Palm: World frame Location')
    print(Palm_in_world)
    print('\n')

    diff_in_world = Palm_in_ArUco - Palm_in_world
    diff_in_world[-1] = 1
    print(diff_in_world)
    np.savetxt('Diff_in_world_frame_of_Palm.csv',diff_in_world, delimiter=',')

    loc = np.array([loc[0], loc[1], loc[2], loc[3]])
    print("loc: ", loc)
    return Palm_in_world, Palm_in_ArUco