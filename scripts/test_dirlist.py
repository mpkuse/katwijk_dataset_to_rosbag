#!/usr/bin/env python

import glob
import os

DB_PATH = "/home/manohar/workspace/bags/external_slam_datasets/katwijk-beach/Part1/"


l=0
n_prev = 0
for file_full in sorted( glob.glob(DB_PATH+'/Velodyne/Velodyne_*_azimuth.png') ):
    file_sp = file_full.split( '/') #eg: katwijk-beach/Part1//Velodyne/Velodyne_2015_11_26_12_57_44_792_azimuth.png
    fname_az = file_sp[-1] #eg: Velodyne_2015_11_26_12_57_44_792_azimuth.png

    fname_stamp = fname_az.split( '_' )[1:-1] #eg: '2015', '11', '26', '12', '55', '36', '786
    fname_stamp_str = '_'.join( fname_stamp ) #eg: 2015_11_26_12_58_34_289

    n = int(fname_stamp[-1]) + int(fname_stamp[-2])*1000 + int(fname_stamp[-3])*100000

    print l, fname_stamp_str, n,
    if n > n_prev:
        print 'OK'
    else:
        print "NOT OK"
        exit(2)
    l+=1
    n_prev = n
