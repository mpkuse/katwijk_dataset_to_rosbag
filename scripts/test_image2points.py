#!/usr/bin/env python

import cv2
import numpy as np
import code

import glob
import os
from datetime import datetime


import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs


class KatwijkUtils:

    def __init__(self, DB_PATH):

        self.DB_PATH = DB_PATH
        self.printing = False



    def _print_opencv_array_info(self, msg, X ):
        if self.printing == False:
            return

        min = np.min(X[:])
        max = np.max(X[:])

        print msg, ": shape=", X.shape, '\t dtype=' , X.dtype,
        print 'min=', min, '\tmax=', max


    def image2points_velodyne(self, time_str):

        # DB_PATH = "/home/manohar/workspace/bags/external_slam_datasets/katwijk-beach/Part1/"
        # time_str = '2015_11_26_12_53_34_667'

        sensor = 'Velodyne'
        ImageLoc_prefix = self.DB_PATH+'/'+sensor+'/'+sensor+"_"+time_str
        color =1

        ########## Load the 3 images
        # Azimuth Image
        azimuthImage_fname = ImageLoc_prefix+'_azimuth.png'
        azimuthImage = cv2.imread(azimuthImage_fname, cv2.IMREAD_UNCHANGED);
        if azimuthImage is None:
            print 'ERROR: Cannot imread: ' , azimuthImage_fname
            print '....QUIT'
            exit(1)
        self._print_opencv_array_info( 'azimuthImage', azimuthImage)

        # Range Image
        rangeImage_fname =  ImageLoc_prefix+'_range.png'
        rangeImage = cv2.imread( rangeImage_fname, cv2.IMREAD_UNCHANGED )
        if rangeImage is None:
            print 'ERROR: Cannot imread: ' , rangeImage_fname
            print '....QUIT'
            exit(1)
        self._print_opencv_array_info( 'rangeImage', rangeImage)



        # Intensity Image
        intensityImage_fname =  ImageLoc_prefix+'_intensity.png'
        intensityImage = None
        if color == 1:
            intensityImage = cv2.imread( intensityImage_fname, cv2.IMREAD_UNCHANGED )
            if intensityImage is None:
                print 'ERROR: Cannot imread: ' , intensityImage_fname
                print '....QUIT'
                exit(1)
            self._print_opencv_array_info( 'intensityImage', intensityImage)


        ########## Angles
        inc_angles = np.expand_dims( range(15,-16,-2) , 0 ).astype('double')
        inc = np.tile( np.transpose(inc_angles), (1,1808) )
        if self.printing == True:
            print "inc_angles.shape=", inc_angles.shape
            print "inc.shape=", inc.shape


        ########## type conversion
        azimuthImage = azimuthImage.astype('double') / 100.
        rangeImage = rangeImage.astype('double')
        if color == 1:
            intensityImage = intensityImage.astype('double')


        ##########
        d = 3
        if color == 1:
            d = 4
        point_cloud = np.zeros(  (rangeImage.shape[0], rangeImage.shape[1], d), np.double )
        point_cloud[ :,:, 2 ] = 0.001 * rangeImage * np.sin( inc/180.*np.pi )
        point_cloud[ :,:, 0 ] = 0.001 * rangeImage * np.cos( inc/180.*np.pi ) * np.cos(azimuthImage/180.*np.pi)
        point_cloud[ :,:, 1 ] = 0.001 * rangeImage * np.cos( inc/180.*np.pi ) * np.sin(azimuthImage/180.*np.pi)

        if color == 1:
            point_cloud[:,:,3] = intensityImage;



        #import scipy.io
        # print 'point_cloud.shape=', point_cloud.shape
        #scipy.io.savemat('/home/manohar/test.mat', {'mydata': point_cloud})
        #code.interact( local=locals() )

        return point_cloud

    def _to_point_cloud(self, points, parent_frame='horizontal_lidar'):
        """ Creates a point cloud message.
        Args:
            points: Nx7 array of xyz positions (m) and rgba colors (0..1)
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """
        print '[_to_point_cloud] points.shape=', points.shape
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = points.astype(dtype).tobytes()

        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate([ 'x', 'y', 'z', 'intensity'] ) ]

        header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

        return sensor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 7),
            row_step=(itemsize * 7 * points.shape[0]),
            data=data
        )


    def image2pointcloudmsg_velodyne( self, time_str):
        point_cloud = self.image2points_velodyne(time_str)

        # convert to point cloud xyzi msg
        point_cloud_flat =  point_cloud.reshape( -1, 4 )
        msg = self._to_point_cloud( point_cloud_flat )

        # todo set time
        try:
            datetime_object = datetime.strptime(time_str, '%Y_%m_%d_%H_%M_%S_%f' )# '%m/%d/%y %H:%M:%S')
            secs = (datetime_object-datetime.utcfromtimestamp(0)).total_seconds()
            rostime = rospy.Time.from_sec( secs )
        except:
            print 'ERROR parsing time_str=', time_str, '   failed'
            exit(1)

        msg.header.frame_id = 'horizontal_lidar'
        msg.header.stamp = rostime

        return msg



# time_str to ros::Time
if __name__ == '__main__':
    time_str = '2015_11_26_12_53_34_667'



    try:
        datetime_object = datetime.strptime(time_str, '%Y_%m_%d_%H_%M_%S_%f' )# '%m/%d/%y %H:%M:%S')
        secs = (datetime_object-datetime.utcfromtimestamp(0)).total_seconds()
        rostime = rospy.Time.from_sec( secs )
    except:
        print 'ERROR parsing time_str=', time_str, '   failed'
        exit(1)
    print datetime_object
    print secs
    print rostime




# basic testing
if __name__ == '__1main__':
    rospy.init_node('talker', anonymous=True)

    DB_PATH = "/home/manohar/workspace/bags/external_slam_datasets/katwijk-beach/Part1/"
    obj = KatwijkUtils(DB_PATH)

    time_str = '2015_11_26_12_53_34_667'
    point_cloud = obj.image2points_velodyne(time_str)
    print '[main] point_cloud.shape=', point_cloud.shape, '\tsum=', np.sum( point_cloud[:])



    # request point cloud msg
    msg = obj.image2pointcloudmsg_velodyne( time_str )

    code.interact( local=locals() )



# with fname
if __name__ == '__m1ain__':
    DB_PATH = "/home/manohar/workspace/bags/external_slam_datasets/katwijk-beach/Part1/"
    obj = KatwijkUtils(DB_PATH)

    l = 0
    for file_full in sorted( glob.glob(DB_PATH+'/Velodyne/Velodyne_*_azimuth.png') ):
        # file_full : eg: katwijk-beach/Part1//Velodyne/Velodyne_2015_11_26_12_57_44_792_azimuth.png
        file_sp = file_full.split( '/')
        fname_az = file_sp[-1] #eg: Velodyne_2015_11_26_12_57_44_792_azimuth.png
        fname_stamp = fname_az.split( '_' )[1:-1] #eg: '2015', '11', '26', '12', '55', '36', '786
        fname_stamp_str = '_'.join( fname_stamp ) #eg: 2015_11_26_12_58_34_289


        print '[main]fname_stamp_str: ' , fname_stamp_str
        point_cloud = obj.image2points_velodyne(fname_stamp_str)
        print '[main] point_cloud.shape=', point_cloud.shape, '\tsum=', np.sum( point_cloud[:])

        l+=1

    print 'Total Loaded: ', l
