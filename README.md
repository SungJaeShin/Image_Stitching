# Image Stitching
## OpenCV based Image Stitching applied various method in C++

## 1. Prerequisites
### 1.1 Dependencies
OpenCV 4.8.0, OpenCV Contrib 4.8.0, C++ 11 version

### 1.2 **Ubuntu** and **ROS**
Follow [ROS Installation](http://wiki.ros.org/ROS/Installation)
- This code tested Ubuntu 64-bit 18.04 (ROS Melodic Version).

### 1.3. **OpenCV Installation**
Follow [OpenCV](https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html)
- Install appropriate OpenCV version: [Here](https://heathered-freon-621.notion.site/Opencv-How-to-install-appropriate-OpenCV-version-86275642fc924df5b1c258f077a94387).

### 1.4 **Intel RealSense ROS package and SDK**
Follow [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

## 2. Get Custom Dataset
<img src="./HW_configuration.png" width = 60% height = 60% div align=center />

Follow [KAIST-DP](https://github.com/SungJaeShin/KAIST-DP.git) Dataset
  - Intel Realsense D455 x3
  - Xsens MTI-300 IMU
  - Velodyne LiDAR VLP 16



### 1.5 **Get KAIST-DP Dataset**
Follow [KAIST-DP]

2. Methodology to apply feature matching
  - [BASIC] Use this Github Page [Feature Matching](https://github.com/SungJaeShin/Feature_matching)
  - [ADD] [Optical Flow Method](https://docs.opencv.org/3.4/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323) 

https://github.com/SungJaeShin/Parallax.git
### 1.6 **Other baseline Codes**
[1] Github: [Feature Matching](https://github.com/SungJaeShin/Feature_matching.git)
[2] Github: [Feature Matching](https://github.com/SungJaeShin/Feature_matching.git)


## 2. Build Stitching_Image
Clone the repository and catkin_make:
```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/SungJaeShin/Stitching_Image.git
    $ cd ../
    $ catkin build
    $ source ~/catkin_ws/devel/setup.bash
```

## 3. Run two cameras
```
    $ roslaunch image_stitching multi_cam.launch
    $ rosrun image_stitching stitching_node
    $ (option) rviz 
```

## 4. Save panorama image in your PC
Change six variables "**save_img1_dir, save_img2_dir, save_img3_dir, save_orb_dir, save_sync_dir, save_pano_dir**" in makePanoramaImage function !!
- When the flag **SAVE_IMGS_WITH_FEATURES** is **1**, then we can visualize feature points of each camera image in **save_orb_dir** !!
- Whem the flag **SAVE_SYNC_IMG_WITH_TIME** is **1**, then we can visualize time stamp of each camera image in **save_sync_dir** !!

## 5. Stitch Result 
|Method| # of Image Stitch|
|:---|:---:|
|Stitch (+ Camera Intrinsic)|56|
|Stitch (+ Camera Intrinsic + Histogram Equalization)|340|
|Stitch (+ Camera Intrinsic + Histogram Equalization + Image Masking)|779|

- In the last method, the panorama outlier rejection method is additionally included.

## 6. BUG to solve later
Related with OpenCV following as:
- OpenCV Error: Assertion failed (features1.descriptors.type() == features2.descriptors.type()) in match 
