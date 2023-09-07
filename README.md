# Repos for Lidar-Camera Calibration

I tried two repositories for calibration: targetless (extended_lidar_calib) and target based (velo2cam).
The target based method needs a large target and is bit more tricky to run and I focused on the targetless method

# Targetless Calibration

The detailed steps to run the calibration is found the in the respective package. Only the
results are shown here.

### Pointcloud with Edge Detections

![](/pictures/pointcloud_with_detected_edges.png)

### Initial and Rough Optimization Results

![](/pictures/initial.png)

![](/pictures/rough_optimization.png)

# Final Results

![](/pictures/colored_cloud.gif)

# Coordinate Frames for Reference

**RealSense**

![](/pictures/RealSense_Frame.png)

**VLP**

![](pictures/vlp_coordinate_frame.jpg)