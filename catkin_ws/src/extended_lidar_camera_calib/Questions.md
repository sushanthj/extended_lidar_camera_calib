# Discussions for the week

1. Where is odometry/filtered coming from?
2. We need better bags, but moreso I need static obstacles which are 3D planar rich
3. I've found few target based methods we could try (mentioned below)
4. They are using odometry estimation for scan-matching, but I don't think that's too important for now
5. I need camera intrinsics (distortion coefficients and intrinsics) even for the extended_lidar_camera_calib
   (right now it's empty in camera_info topic)


# [External Lidar-Camera Calib (Shrijit's suggestion)](https://github.com/sushanthj/extended_lidar_camera_calib) Performance and Results

- The accumulated pointcloud was not feature rich enough (this may have influenced the calibration too)
  ![](/extended_lidar_camera_calib/pics/shrijit_calib_1.png)

- The results of calibration may have been affected by intrinsics needed which were not set
  ![](/extended_lidar_camera_calib/pics/shrijit_calib_2.png) 


# Target-Less Methods

All the ones below need basic intrinsics to be known.

1. [External Lidar-Camera Calib (Shrijit's suggestion)](https://github.com/sushanthj/extended_lidar_camera_calib)
   Essentially a combination of FLOAM and Livox
2. [Open Calib's lidar2camera](https://github.com/PJLab-ADG/SensorsCalibration/blob/master/lidar2camera/README.md)
   However, this needs an extrinsics estimate

# Target Based Methods

1. [Joint Camera Intrinsic and Lidar2Cam calib](https://github.com/OpenCalib/JointCalib/tree/master)
   Seems feasible in ros1, but poor documentation and board seems bit hard to make although CSV seems flexible
2. [velo2cam calibration](https://github.com/beltransen/velo2cam_calibration)
   The board for this seems easier (just need large carboard and 4 A4 sheets for aruco tags)
3. [Lidar Reflectance based method, might not work in field, also untested on VLP16](https://github.com/mfxox/ILCC)
4. [Manul calib](https://github.com/ankitdhall/lidar_camera_calibration#usage)
   Good documentation and seems to work on noetic. We can try


# velo2cam

1. All objects must be static for at least 30 frames
2. The calibration board as well **as the surface behind it** should be visible
3. The frame_id of the sensor_msgs/Image topic and the camera_calibration topic must have the same frame ID
4. The camera_calibration topic must have the distortion parameters
5. Pattern should be kept in FOV of both Lidar and Mono Camera


## Pipeline build

This was simple and the only packages required in noetic was:
```
sudo apt install ros-noetic-opencv-apps
sudo apt install ros-noetic-tf-conversions
sudo apt install ros-noetic-image-geometry
```

## Issues with current rosbag

### oak/rgb/camera_info and oak/rgb/image_raw

The topics for this camera seem perfect with distortion parameters filled
as well as the frame_id for both the topics matching (which is required)

![](/extended_lidar_camera_calib/pics/oak_image_raw_header.png)

![](/extended_lidar_camera_calib/pics/oak_camera_info.png)

However, it looks like this was not the camera inteded for calibration (target not in view)

![](/extended_lidar_camera_calib/pics/oak_image_view.png)

### Bayer Camera

This topic did not have any distortion parameters in the camera info (will need to
be filled if it is to be used). The frame ID will also need to be manually set to
satisfy the velo2cam requirements

![](/extended_lidar_camera_calib/pics/bayer_image_raw_header.png)

![](/extended_lidar_camera_calib/pics/bayer_camera_info.png)

The target in the rosbag also has to be stationary, in the rosbag it was handheld,
will need to capture another rosbag in FRC.

![](/extended_lidar_camera_calib/pics/bayer_camera.png)