#! /usr/bin/env python

import os

import numpy as np
import csv
import glob
import sys



dir_path = os.path.dirname(os.path.realpath(__file__))
marker_dict = {}

with open(dir_path + '/final_test/data_file_6.csv') as f:
            reader = csv.reader(f)
            next(reader)
            for j, row in enumerate(reader):
                rotation_mat = []
                for i in range(1,len(row)):
                    rotation_mat.append(float(row[i]))
                
                # print(row[0])
                marker_dict[str(row[0])] = rotation_mat

print(marker_dict)