# Kitti Eigen split camera-lidar visualizer
A simple c++ tool to visualize (synchronized) kitti gray images and Lidar point cloud together. May be used for camera-lidar synchronization debugging.

Libs: PCL(1.13.1), OpenCV(4.3.0)


![alt text](https://github.com/QtSignalProcessing/cam_lidar_visualizer/blob/main/res/screenshot.png)


### TODO
- [ ] Define a virtual class Dataset for easier extension
- [ ] Dataloader for standard Kitti data storage structure
- [ ] Lidar-cam calibration visualization
- [x] Two view disparity estimation
- [ ] Add comment 
- [ ] simple GUI?