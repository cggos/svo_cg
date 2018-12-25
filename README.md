# SVO

from https://github.com/uzh-rpg/rpg_svo, implement a **Semi-direct Monocular Visual Odometry** pipeline.

-----

## Dependencies
* **Boost** - C++ Librairies (thread and system are needed)
* **Eigen 3** - Linear algebra
* **OpenCV** - Computer vision library for loading and displaying images
* **[Sophus](https://github.com/strasdat/Sophus.git)** - Lie groups
  ```bash
  git checkout a621ff
  ```
* **[Fast](https://github.com/uzh-rpg/fast.git)** - Corner Detector
* **[g2o](https://github.com/RainerKuemmerle/g2o.git)** - General Graph Optimization OPTIONAL
  ```bash
  https://github.com/RainerKuemmerle/g2o/archive/20160424_git.tar.gz
  
  export G2O_ROOT=$HOME/installdir
  ```
* **[vikit_common](https://github.com/uzh-rpg/rpg_vikit.git)** (git submodule)

## Run SVO

### with ROS

#### on a Dataset

```bash
roslaunch svo_ros test_rig3.launch
rviz -d svo_ros/rviz_config.rviz
rosbag play airground_rig_s3_2013-03-18_21-38-48.bag
```

# [SVO 2.0](http://rpg.ifi.uzh.ch/svo2.html)
SVO 2.0: Fast Semi-Direct Visual Odometry for Monocular, Wide Angle, and Multi-camera Systems.  

-----

SVO 2.0 (IEEE TRO'17) extends the original SVO impleemntation (ICRA' 14) with the addition of edgletes, IMU prior, wide angle cameras (fisheye and catadioptric), multi-camera configurations, and forward looking camera motion.
