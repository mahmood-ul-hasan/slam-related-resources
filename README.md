# slam-related-resources
A collection of SLAM, odometry methods, and related resources frequently referenced in robotics and ROS research. This repository includes various algorithms, tools, and datasets for 2D/3D LiDAR, visual-inertial odometry, and feature-based SLAM implementations, with a focus on ROS and ROS2 compatibility.


# SLAM and Related Methods Often Mentioned in Papers


## Overview
Although I mentioned in the title that this will be a comparison, I will simply highlight things that interest me. This document will be updated periodically.

### List (As of April 2024)

---

### Terminology

- **LIO (LiDAR Inertial Odometry):** A method where LiDAR and IMU are tightly coupled.
- **VIO (Visual Inertial Odometry):** A method where visual (image) and IMU are tightly coupled.
- **LiDAR Odometry/SLAM**

---

### LiDAR-based SLAM/Odometry

- **[GMapping](https://github.com/ros-perception/slam_gmapping)**  
  License: BSD-3  
  Keywords: Rao-Blackwellised Particle Filter, 2D LiDAR, ROS

- **[HDL Graph SLAM](https://github.com/koide3/hdl_graph_slam)**  
  License: BSD-2  
  Keywords: Graph-based SLAM, 3D LiDAR, NDT/ICP/GICP/VGICP, people tracking, ROS  
  Multiple scan matching methods, IMU and floor constraints support, separate localization and people tracking packages.

- **[LOAM](https://github.com/laboshinl/loam_velodyne)**  
  License: BSD  
  Keywords: 3D LiDAR, ROS  
  Plane and edge detection matching, real-time by separating odometry and mapping.

- **[ALOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)**  
  License: BSD  
  Keywords: 3D LiDAR, ROS

- **[FLOAM](https://github.com/wh200720041/floam)**  
  License: BSD  
  Keywords: 3D LiDAR, ROS

- **[LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)**  
  License: BSD-3  
  Keywords: 3D LiDAR, ROS

- **[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)**  
  License: BSD-3  
  Keywords: 3D LiDAR, LIO, ROS1/ROS2  
  First tightly coupled LiDAR and IMU method. ROS2 branch available.

- **[LILI-OM](https://github.com/KIT-ISAS/lili-om)**  
  License: GPL-3.0  
  Keywords: 3D LiDAR, LIO, ROS  
  Feature extraction for irregular LiDAR scan patterns with sliding window keyframe selection.

- **[LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)**  
  Keywords: 3D LiDAR, LIO, ROS  
  Uses 6-DOF attitude estimation via iterative error-state Kalman filter (ESKF).

- **[FAST-LIO/FAST-LIO2](https://github.com/hku-mars/FAST_LIO)**  
  License: GPL-2.0  
  Keywords: 3D LiDAR, LIO, ikd-tree, ROS  
  FAST-LIO: Iterative Kalman filter and Kalman gain formula for tightly coupled LiDAR and IMU.  
  FAST-LIO2: Fast search using ikd-tree. Mapping without feature extraction.

- **[LOCUS](https://github.com/NeBula-Autonomy/LOCUS)**  
  Keywords: Not provided

- **[Kiss-ICP](https://github.com/PRBonn/kiss-icp)**  
  Keywords: Not provided

- **[CT-ICP](https://github.com/jedeschaud/ct_icp)**  
  Keywords: Not provided

- **[Faster-LIO](https://github.com/gaoxiang12/faster-lio)**  
  Keywords: Not provided

- **[Point-LIO](https://github.com/hku-mars/Point-LIO)**  
  Keywords: Not provided

- **[VoxelMap](https://github.com/hku-mars/VoxelMap)**  
  Keywords: Not provided

- **[VoxelMap++](https://github.com/uestc-icsp/VoxelMapPlus_Public)**  
  Keywords: Not provided

- **[DLO (Direct LiDAR Odometry)](https://github.com/vectr-ucla/direct_lidar_odometry)**  
  Keywords: Not provided

- **[DLIO (Direct LiDAR Inertial Odometry)](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)**  
  Keywords: ROS2 branch available

- **[Intensity-based LiDAR SLAM](https://github.com/MISTLab/Intensity_based_LiDAR_SLAM)**  
  Keywords: Not provided

- **[DMSA_LiDAR_SLAM](https://github.com/davidskdds/DMSA_LiDAR_SLAM)**  
  Keywords: Not provided

- **[MD-SLAM](https://github.com/rvp-group/mdslam)**  
  Keywords: How LiDAR and RGB-D are processed in the same pipeline

- **[BA-MDSLAM (Improved version of MD-SLAM)](https://github.com/rvp-group/ba-mdslam)**  
  Keywords: Not provided

- **[NeRF-LOAM](https://github.com/JunyuanDeng/NeRF-LOAM)**  
  Keywords: NeRF + LOAM integration

  

---

### Visual Odometry/SLAM

- **[SVO](https://github.com/uzh-rpg/rpg_svo)**  
  License: GPL-3.0  
  Keywords: Direct-based, Monocular

- **[SVO 2.0](https://github.com/uzh-rpg/rpg_svo_pro_open)**  
  License: GPL-3.0  
  Keywords: Direct-based, Monocular, Stereo, Fish-Eye

- **[DynaSLAM](https://github.com/BertaBescos/DynaSLAM)**  
  License: CC BY-NC 4.0  
  Keywords: Monocular, Stereo, RGB-D

- **[ORB-SLAM](https://github.com/raulmur/ORB_SLAM)**  
  License: GPL3.0  
  Keywords: Feature-based, Monocular

- **[ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)**  
  License: GPL3.0  
  Keywords: Feature-based, Monocular, Stereo, RGB-D  
  ROS2 package available: [ORB-SLAM2 ROS2](https://github.com/appliedAI-Initiative/orb_slam_2_ros/tree/ros2)

- **[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)**  
  License: GPL3.0  
  Keywords: Feature-based, Monocular, Stereo, RGB-D, Fish-Eye, IMU

  - **[OpenVINS](https://github.com/rpng/open_vins)**  
  License: GPL-3.0  
  Keywords: Feature-based, ROS1/ROS2

- **[VDO-SLAM](https://github.com/halajun/VDO_SLAM)**  
  License: GPL-3.0  
  Keywords: RGB-D, Dynamic SLAM

- **[DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM)**  
  License: BSD-3  
  Keywords: DNN-based, Monocular, Stereo, RGB-D

- **[OKVIS](https://github.com/ethz-asl/okvis)**  
  License: BSD-3  
  Keywords: Feature-based

- **[Cube-SLAM](https://github.com/shichaoy/cube_slam)**  
  License: BSD-3  
  Keywords: Feature-based, ORB-SLAM2-based, Monocular, 2D/3D Object Detection

- **[QuadricSLAM](https://github.com/qcr/quadricslam)**  
  License: BSD-3  
  Keywords: Feature-based, ORB-SLAM2-based

- **[VINS-mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)**  
  License: GPL-3.0  
  Keywords: Feature-based, VIO, Monocular

- **[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)**  
  License: GPL-3.0  
  Keywords: Feature-based, VIO, Monocular, Stereo

- **[EVO](https://github.com/uzh-rpg/rpg_dvs_evo_open)**  
  License: Patented  
  Keywords: Event-based Camera

- **[ElasticFusion](https://github.com/mp3guy/ElasticFusion)**  
  License: Proprietary (Non-commercial use only)

- **[NICE-SLAM](https://github.com)**  
  Keywords: Not provided

- **[Point-SLAM](https://github.com/eriksandstroem/Point-SLAM)**  
  License: Apache-2.0  
  Keywords: Not provided

- **[Gaussian Splatting SLAM](https://github.com/muskie82/MonoGS)**  
  Keywords: Not provided


---

### Fusion

- **[LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)**  
- **[R2LIVE](https://github.com/hku-mars/r2live)**  
  License: GPL-2.0
- **[R3LIVE](https://github.com/hku-mars/r3live)**  
  License: GPL-2.0
- **[FAST-LIVO](https://github.com/hku-mars/FAST-LIVO)**  
  License: GPL-2.0

---

### Pose Graph Optimization

- **[Ceres Solver](http://ceres-solver.org/)**  
  License: BSD
- **[g2o](https://github.com/RainerKuemmerle/g2o)**  
  License: BSD (some dependent modules are LGPL)
- **[gtsam](https://gtsam.org/)**  
  License: BSD

---

### Loop Closure

- **[DBoW2](https://github.com/dorian3d/DBoW2)**  
  Algorithm used in ORB-SLAM

- **[Scan-Context](https://github.com/irapkaist/scancontext)**  
  License: CC BY-NC-SA 4.0  
  The repository has been moved. See the advanced version, [Scan-Context++](https://github.com/gisbi-kim/scancontext_tro)

- **[LiDAR-Iris](https://github.com/BigMoWangying/LiDAR-Iris)**  
  License: MIT

- **[M2DP](https://github.com/LiHeUA/M2DP)**  
  License: Unknown

- **[Intensity Scan Context](https://github.com/wh200720041/iscloam)**  
  License: BSD

- **[OverlapNet](https://github.com/PRBonn/OverlapNet)**  
  License: MIT

---

### Scan Matching

- **NDT, ICP, GICP**  
  Implemented in PCL

- **[NDT-OMP](https://github.com/koide3/ndt_omp)**  
  For the ROS2 version, use [this](https://github.com/tier4/ndt_omp)

- **[FAST-GICP](https://github.com/SMRT-AIST/fast_gicp)**

- **[Nano-GICP](https://github.com/engcang/nano_gicp)**  
  Nano-GICP is separated from DLO as a module.

- **FAST-GICP + Nano-FLANN**

- **[small_gicp](https://github.com/koide3/small_gicp)**  
  Up to twice as fast as FAST-GICP

---

### Object Detection

- **[CenterPoint](https://github.com/tianweiy/CenterPoint)**

- **Mink**

- **[YOLOv3](https://pjreddie.com/darknet/yolo/)**

- **[SSD](https://github.com/amdegroot/ssd.pytorch)**

---

### Semantic Segmentation

---

### Depth Estimation

- **[monodepth](https://github.com/mrharicot/monodepth)**  
  License: UCLA ACP-A

- **[monodepth2](https://github.com/nianticlabs/monodepth2)**  
  License: monodepth2 (proprietary)

---

### Feature Detection

- **SIFT, SURF, ORB**  
  Implemented in OpenCV

- **[SuperPoint](https://github.com/rpautrat/SuperPoint)**  
  License: MIT  
  Keywords: CNN

- **[cuda-efficient-features](https://github.com/fixstars/cuda-efficient-features)**

---

### Feature Matching

- **[SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork)**  
  License: Proprietary  
  Keywords: SuperPoint, Attention Graph Neural Network

- **[LightGlue](https://github.com/cvg/LightGlue)**  
  License: Apache-2.0

---

### Structure from Motion

- **[COLMAP](https://github.com/colmap/colmap)**  
  License: BSD  
  Often used for camera parameter estimation

---

### 3D Reconstruction

- **[NeRF](https://github.com/bmild/nerf)**  
  License: MIT  
  Keywords: Machine Learning

- **[3D Gaussian-Splatting](https://github.com/graphdeco-inria/gaussian-splatting)**  
  License: Proprietary

---

### Tools

- **[pg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation)**  
  VIO trajectory evaluation tool

- **[evo](https://github.com/MichaelGrupp/evo)**  
  Package for SLAM evaluation

- **[rosbag2_bag_v2](https://github.com/ros2/rosbag2_bag_v2)**  
  Convert ROS1 rosbag to ROS2 rosbag

- **[rosbags](https://github.com/rpng/rosbags)**

---

### Dataset

- **[Hilti Challenge Dataset](https://hilti-challenge.com/)**

- **[New College Dataset](https://ori-drs.github.io/newer-college-dataset/)**


