#!/usr/bin/env python

import numpy as np
import csv


DB_PATH = "/home/manohar/workspace/bags/external_slam_datasets/katwijk-beach/Part1/"
imu_fname = DB_PATH + '/imu.txt'
with open(imu_fname, 'r') as file:
    reader = csv.reader(file,  delimiter =' ')
    for row in reader:
        print(row[0])
