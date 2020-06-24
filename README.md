# Katwijk dataset to rosbag
This utility can be used to convert katwijk dataset to rosbag.
Katwijk dataset is distributed as a bunch of image files and csv files.

## Katwijk Beach Planetary Rover Dataset
The official website: https://robotics.estec.esa.int/datasets/katwijk-beach-11-2015/


## How to run
clone this pkg to your catkin_ws. In `scripts/to_rosbag.py` edit the
DB_PATH. This is the location of the dataset.

```
rosrun katwijk_dataset_to_rosbag to_rosbag.py
```

## Notes
Currently only dumps the lidar data (as PointCloud2) and imu data to the bag.
Other data to bag will be implemented as soon. Contributions welcome.

## Author
Manohar Kuse
