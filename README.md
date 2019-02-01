# svo_cg

Modified version of [rpg_svo](https://github.com/uzh-rpg/rpg_svo)(commit d616106 on May 9, 2017) which implement a **Semi-direct Monocular Visual Odometry** pipeline

-----

[TOC]

## Build & Run

### Dependencies
* **Boost** - C++ Librairies (thread and system are needed)

* **Eigen 3** - Linear algebra

* **OpenCV** - Computer vision library for loading and displaying images

* **[Sophus](https://github.com/strasdat/Sophus.git)** - Lie groups
  - - recommend version: commit id **a621ff**

* **[Fast](https://github.com/uzh-rpg/fast.git)** - Corner Detector

* **[g2o](https://github.com/RainerKuemmerle/g2o.git)** - General Graph Optimization OPTIONAL
  - recommend version: commit id **ff647bd** (ff647bd7537860a2b53b3b774ea821a3170feb13)
  ```bash
  export G2O_ROOT=$HOME/installdir
  ```
  
* **[rpg_vikit](https://github.com/uzh-rpg/rpg_vikit.git)** (git submodule)


### Install

* install Dependencies
* `git clone https://github.com/cggos/svo_cg.git --recursive`
* `cmake` for Plain CMake (No ROS) or `catkin_make` for ROS

### Run

#### with ROS

* on a Dataset
  ```bash
  roslaunch svo_ros test_rig3.launch
  rviz -d svo_ros/rviz_config.rviz
  rosbag play airground_rig_s3_2013-03-18_21-38-48.bag
  ```
* on a live camera stream (eg. RealSense ZR300 fisheye camera)
  - calibrate your camera and modify `svo_ros/param/camera_atan_zr300.yaml`
  - `roslaunch svo_ros live.launch`
  - `rviz -d svo_ros/rviz_config.rviz`


#### without ROS

* on a Dataset
  ```bash
  export SVO_DATASET_DIR=${HOME}/Datasets  # .bashrc

  source ~/.bashrc
  cd ${SVO_DATASET_DIR}
  wget http://rpg.ifi.uzh.ch/datasets/sin2_tex2_h1_v8_d.tar.gz -O - | tar -xz

  cd svo/bin
  ./test_pipeline
  ```


## Supported Camera Model

* **ATAN model** (preference)
  * which is also used by **PTAM** and uses **the FOV distortion model**
  * calibration tool: **PTAM Calibration**
* **Pinhole model**
  * three radial and two tangential distortion parameters
  * calibration tool: [camera_calibration (ROS)](http://wiki.ros.org/camera_calibration)
* **Ocam model** (by Davide Scaramuzza)
  * model cameras with **high field of view or even omnidirectional cameras**
  * calibration tool: **OCamCalib toolbox**


## Notation

<div align=center>
  <img src="https://raw.githubusercontent.com/uzh-rpg/rpg_svo/master/svo/doc/notation.png">
</div>

* **px** - Pixel coordinate (u,v)
* **f** - Bearing vector of unit length (x,y,z)
* **T_f_w** - Rigid body transformation from world frame **w** to camera frame **f**
* transforms a point in world coordinates **p_w** to a point in frame coordinates **p_f** as follows
  ```sh
  p_f = T_f_w * p_w
  ```
* camera position in world coordinates must be obtained by inversion
  ```sh
  pos = T_f_w.inverse().translation()
  ```

## Obtaining Best Performance
In order to obtain the same performance as shown in the videos, consider the following:

* Use a global shutter camera (we use a grayscale matrix-vision bluefox camera, WVGA resolution).
* Set the framerate of the camera as high as possible (we use 70fps).
* Depending on your camera driver it might be better to manually fix the shutter speed and gain to avoid flickering.
* Use a lens with large field of view (we have approx. 110 deg).
* Calibrate the camera with the ATAN model and make sure you have a very low reprojection error (~0.1px).
* For higher robustness, you can increase the number of tracked features by setting the parameter svo/max_fts to 180.
* Avoid motions of pure rotation.
* The keyframe selection is currently designed for downlooking cameras. Forward motions are not performing well at the moment.

## Notes

* Forward motions
  - The current keyframe selection criterion is designed for downward looking cameras. This is one reason why SVO does not work well for forward motions (e.g., to be used on a car).
* Image resolution
  - The current parameters are tuned for WVGA resolution. If you use a higher resolution camera, then the pyramid levels should be increased as well as the reprojection error thresholds.

# [SVO 2.0](http://rpg.ifi.uzh.ch/svo2.html)
SVO 2.0: Fast Semi-Direct Visual Odometry for Monocular, Wide Angle, and Multi-camera Systems.  

-----

SVO 2.0 (IEEE TRO'17) extends the original SVO impleemntation (ICRA' 14) with the addition of edgletes, IMU prior, wide angle cameras (fisheye and catadioptric), multi-camera configurations, and forward looking camera motion.
