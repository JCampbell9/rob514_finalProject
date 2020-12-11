#! usr/bin/env python3



import numpy as np
import csv
import sys
import os


directory = os.path.dirname(os.path.realpath(__file__))

# sets up matrices to read the stored matrices into
tran_mat = np.zeros((4, 4))
translation_mat = np.zeros((4, 4))
rotation_mat = np.zeros((4, 4))

print("WORLD TO ARM BASE")

########################################################################################################################
# Read in the translation rotation and transformation matrices
########################################################################################################################

with open(directory + '/TranslationMatrix.csv', newline='') as f:
    reader = csv.reader(f)
    for j, row in enumerate(reader):
        for i, col in enumerate(row):
            translation_mat[j][i] = float(col)


with open(directory + '/RotationMatrix.csv', newline='') as f:
    reader = csv.reader(f)
    for j, row in enumerate(reader):
        for i, col in enumerate(row):
            rotation_mat[j][i] = float(col)


with open(directory + '/Test_11_9/TransformMatrix_8.0.csv', newline='') as f:
    reader = csv.reader(f)
    for j, row in enumerate(reader):
        for i, col in enumerate(row):
            tran_mat[j][i] = float(col)


########################################################################################################################
# Calculate the end effector location in the world frame
########################################################################################################################

world_locs = [[0, 0, 0, 1], [0.3, 0.25, 0, 1], [-0.3, 0.25, 0, 1], [-0.3, -0.25, 0, 1], [0.3, -0.25, 0, 1]]
names = ['Center: ', 'Front Left: ', 'Front Right: ', 'Back Right: ', 'Back Left: ']

for i in range(4):

    transform =  rotation_mat @ translation_mat

    loc = np.linalg.inv(transform) @ np.transpose(world_locs[i])

    # print(world_loc)

    # print(np.transpose(transform))

    print(names[i], loc)

