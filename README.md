# extended_lidar_camera_calib

This work is an extended version of [livox_camera_calib](https://github.com/hku-mars/livox_camera_calib.git), which is suitable for spinning LiDAR。

In order to apply this algorithm on spinning LIDAR(e.g:VLP16)， I add the preprocess process([FLOAM](https://github.com/wh200720041/floam.git)) to make the point cloud of the spinning LiDAR denser.

## Data Prepare

When you calibrate the spinning lidar and camera, record the data by holding the device(lidar and camera) stationary for a period of time and then slowly and repeatedly move the entire device in this direction to accumulate the point cloud.

## Dependency
```
ceres-slover == 1.14.0
pcl==1.8.0
```

## Context You'l need to know
1. This repo simply links to two completely separate libraries FLOAM and LIVOX
2. Pleas take a look at those two repos(folders here) individually to understand better

## Setup
1. Create two folders, one called 'input_bags' and the other called 'catkin_ws' (RUN mkdir catkin_ws/src)
2. Inside the src folder of catkin_ws do ```git clone https://github.com/sushanthj/extended_lidar_camera_calib.git```

Now, move the run_docker.sh and setup.sh scripts from within the ```extened_lidar_calib``` folder
and modify it's contents with the correct paths

## Run
1. Start the docker container by running run_docker.sh
2. Run the setup.sh script to get all dependencies setup
3. Go to the catkin_ws and run catkin_make

step1: doing slam to accumulate dense pointcloud. 
```
roslaunch floam floam_XXX.launch
```

This floam_XXX is because there are a few options. If using a VLP16
I think the following should be done:
1. Create a folder called /dataset within the floam folder
2. Put your rosbags there
3. Go to the floam_mapping.launch file and change the path to the rosbag
4. (optional if map.pcd is not saved later) In the same launch file, also change the ```map_save_ROOT``` parameter
5. Run the floam node: ```roslaunch floam floam_mapping.launch```
6. The node does not exit on it's own. But just do CTRL-C and it should save the map smoothly

step2: lidar camera calibration
```
roslaunch livox_camera_calib calib_XXX.launch
```

## My Result
In my test, I get better results in indoor enviroment. 
```
LIDAR : Robosense RS-Bpearl
Camera: MindVision MV-SUA133GC-T
```
slam result:
<p align='center'>
<img width="65%" src="pics/floam_save_pcd.png"/>
</p>

map result:
<p align='center'>
<img width="65%" src="pics/map_pcd.png"/>
</p>

calib result:
<p align='center'>
<img width="65%" src="pics/calib_result.png"/>
</p>

## Acknowledgements
Thanks for [livox_camera_calib](https://github.com/hku-mars/livox_camera_calib.git) and [FLOAM](https://github.com/wh200720041/floam.git).
