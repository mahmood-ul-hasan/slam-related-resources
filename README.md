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

| Year | Short Name | Paper Name                                                              | Sensors           | Comments                                                        | GitHub Link                                                    | Paper Link                                                                  | License  |
|------|------------|-------------------------------------------------------------------------|-------------------|-----------------------------------------------------------------|---------------------------------------------------------------|-----------------------------------------------------------------------------|----------|
| 2024 | GLIM       | GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors | 3D LiDAR, IMU     | GPU-accelerated scan matching factors                          | [GitHub](https://github.com/koide3/glim)                       | [Paper](https://arxiv.org/pdf/2401.12345.pdf)                             | N/A      |
| 2022 | LIO-SAM    | LIO-SAM: Tightly-Coupled Lidar Inertial Odometry via Smoothing and Mapping | 3D LiDAR, IMU     | Tightly coupled LiDAR and IMU, ROS1/ROS2 support              | [GitHub](https://github.com/TixiaoShan/LIO-SAM)               | [Paper](https://arxiv.org/pdf/2007.00258.pdf)                             | BSD-3    |
| 2022 | MD-SLAM    | MD-SLAM: A Framework for Multi-Modal SLAM                              | LiDAR, RGB-D      | Processes LiDAR and RGB-D in the same pipeline                 | [GitHub](https://github.com/rvp-group/mdslam)                 | N/A                                                                         | N/A      |
| 2022 | LeGO-LOAM   | LeGO-LOAM: Lightweight and Ground-Optimized LiDAR Odometry and Mapping  | 3D LiDAR          | Ground-optimized mapping                                        | [GitHub](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) | [Paper](https://arxiv.org/pdf/1804.02903.pdf)                             | BSD-3    |
| 2022 | Faster-LIO | Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels | 3D LiDAR, IMU     | Based on Fast-lio. Kalman filter for tightly coupled LiDAR and IMU                | [GitHub](https://github.com/gaoxiang12/faster-lio)          | [Paper](https://doi.org/10.1109/LRA.2022.3152830)                         | N/A      |
| 2021 | FAST-LIO   | FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package               | 3D LiDAR, IMU     | Kalman filter for tightly coupled LiDAR and IMU                | [GitHub](https://github.com/hku-mars/FAST_LIO)                | [Paper](https://arxiv.org/pdf/2107.05307.pdf)                             | GPL-2.0  |
| 2020 | DLO        | N/A                                                                     | 3D LiDAR          | Direct LiDAR Odometry                                          | [GitHub](https://github.com/vectr-ucla/direct_lidar_odometry) | N/A                                                                         | N/A      |
| 2020 | DLIO       | N/A                                                                     | 3D LiDAR, IMU     | ROS2 branch available                                          | [GitHub](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) | N/A                                                                         | N/A      |
| 2020 | HDL Graph SLAM | Robust and Precise Localization in Urban Environments Using Multi-Layered LiDAR | 3D LiDAR, IMU     | NDT/ICP/GICP/VGICP, people tracking, IMU and floor constraints | [GitHub](https://github.com/koide3/hdl_graph_slam)            | [Paper](https://arxiv.org/pdf/1604.00946.pdf)                             | BSD-2    |
| 2018 | ALOAM      | Advanced LOAM                                                           | 3D LiDAR          | 3D LiDAR, ROS                                                 | [GitHub](https://github.com/HKUST-Aerial-Robotics/A-LOAM)      | [Paper](https://arxiv.org/pdf/1802.02275.pdf)                             | BSD      |
| 2018 | LOAM       | LOAM: Lidar Odometry and Mapping in Real-time                           | 3D LiDAR          | Plane and edge detection, real-time by separating odometry and mapping | [GitHub](https://github.com/laboshinl/loam_velodyne)           | [Paper](https://ieeexplore.ieee.org/document/6907033)                     | BSD      |
| 2018 | NeRF-LOAM  | N/A                                                                     | 3D LiDAR, NeRF    | NeRF + LOAM integration                                         | [GitHub](https://github.com/JunyuanDeng/NeRF-LOAM)             | N/A                                                                         | N/A      |
| 2018 | LeGO-LOAM  | LeGO-LOAM: Lightweight and Ground-Optimized LiDAR Odometry and Mapping  | 3D LiDAR          | Ground-optimized mapping                                        | [GitHub](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) | [Paper](https://arxiv.org/pdf/1804.02903.pdf)                             | BSD-3    |
| 2017 | LILI-OM    | N/A                                                                     | 3D LiDAR          | Feature extraction for irregular LiDAR scan patterns           | [GitHub](https://github.com/KIT-ISAS/lili-om)                  | N/A                                                                         | GPL-3.0  |
| 2016 | GMapping   | Improved Techniques for Grid Mapping With Rao-Blackwellized Particle Filters | 2D LiDAR          | Rao-Blackwellised Particle Filter, ROS                         | [GitHub](https://github.com/ros-perception/slam_gmapping)     | [Paper](https://www2.informatik.uni-freiburg.de/~grisetti/pdf/grisetti07ijrr.pdf) | BSD-3    |
| N/A  | Point-LIO  | N/A                                                                     | 3D LiDAR, IMU     | Not provided                                                  | [GitHub](https://github.com/hku-mars/Point-LIO)                | N/A                                                                         | N/A      |
| N/A  | VoxelMap   | N/A                                                                     | Not provided      | Not provided                                                  | [GitHub](https://github.com/hku-mars/VoxelMap)                 | N/A                                                                         | N/A      |
| N/A  | VoxelMap++ | N/A                                                                     | Not provided      | Not provided                                                  | [GitHub](https://github.com/uestc-icsp/VoxelMapPlus_Public)    | N/A                                                                         | N/A      |
| N/A  | Intensity-based LiDAR SLAM | N/A                                                 | Not provided      | Not provided                                                  | [GitHub](https://github.com/MISTLab/Intensity_based_LiDAR_SLAM) | N/A                                                                         | N/A      |
| N/A  | DMSA_LiDAR_SLAM | N/A                                                            | Not provided      | Not provided                                                  | [GitHub](https://github.com/davidskdds/DMSA_LiDAR_SLAM)        | N/A                                                                         | N/A      |
| N/A  | BA-MDSLAM (Improved version of MD-SLAM) | N/A                                         | Not provided      | Improved version of MD-SLAM                                    | [GitHub](https://github.com/rvp-group/ba-mdslam)               | N/A                                                                         | N/A      |
| N/A  | LOCUS      | N/A                                                                     | Not provided      | Not provided                                                  | [GitHub](https://github.com/NeBula-Autonomy/LOCUS)             | N/A                                                                         | N/A      |
| N/A  | Kiss-ICP   | N/A                                                                     | Not provided      | Not provided                                                  | [GitHub](https://github.com/PRBonn/kiss-icp)                   | N/A                                                                         | N/A      |
| N/A  | CT-ICP     | N/A                                                                     | Not provided      | Not provided                                                  | [GitHub](https://github.com/jedeschaud/ct_icp)                 | N/A                                                                         | N/A      |


---

### Visual Odometry/SLAM

| Year | Short Name      | Paper Name                                                                                                                    | Sensors                     | Comments                                         | GitHub Link                                                             | Paper Link                                                                  | License             |
|------|-----------------|------------------------------------------------------------------------------------------------------------------------------|-----------------------------|-------------------------------------------------|-------------------------------------------------------------------------|-----------------------------------------------------------------------------|---------------------|
| 2011 | ORB-SLAM        | [ORB-SLAM: a Versatile and Accurate Monocular SLAM System](https://arxiv.org/abs/1502.00956)                              | Monocular                   | Foundational work in feature-based SLAM.       | [GitHub](https://github.com/raulmur/ORB_SLAM)                         | [Paper](https://arxiv.org/abs/1502.00956)                                   | GPL3.0              |
| 2017 | ORB-SLAM2       | [ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras](https://arxiv.org/abs/2007.00732)         | Monocular, Stereo, RGB-D    | Improved performance and new features.         | [GitHub](https://github.com/raulmur/ORB_SLAM2)                         | [Paper](https://arxiv.org/abs/2007.00732)                                   | GPL3.0              |
| 2020 | ORB-SLAM3       | [ORB-SLAM3: An Accurate Open-Source Library for SLAM](https://arxiv.org/abs/2007.03054)                                  | Monocular, Stereo, RGB-D, Fish-Eye, IMU | State-of-the-art performance.                  | [GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)                      | [Paper](https://arxiv.org/abs/2007.03054)                                   | GPL3.0              |
| 2018 | VINS-Mono       | [VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator](https://ieeexplore.ieee.org/document/8451184) | Monocular                   | Robust performance in various conditions.      | [GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)           | [Paper](https://ieeexplore.ieee.org/document/8451184)                      | GPL-3.0             |
| 2020 | VINS-Fusion     | [VINS-Fusion: A Versatile and Robust Visual-Inertial State Estimator](https://arxiv.org/abs/2008.10565)                    | Monocular, Stereo           | Integration of various sensors.                | [GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)         | [Paper](https://arxiv.org/abs/2008.10565)                                   | GPL-3.0             |
| 2020 | OKVIS           | [OKVIS: Open Keyframe-based Visual-Inertial SLAM](https://arxiv.org/abs/1803.00389)                                      | Monocular, Stereo           | Real-time performance with loop closure.       | [GitHub](https://github.com/ethz-asl/okvis)                            | [Paper](https://arxiv.org/abs/1803.00389)                                   | BSD-3               |
| 2017 | SVO             | [SVO: Semi-direct Visual Odometry for Monocular and Multi-camera Systems](https://arxiv.org/abs/1607.06501)                | Monocular                   | Semi-direct method for improved tracking.      | [GitHub](https://github.com/uzh-rpg/rpg_svo)                            | [Paper](https://arxiv.org/abs/1607.06501)                                   | GPL-3.0             |
| 2020 | DynaSLAM        | [DynaSLAM: Tracking, Mapping, and Inference in Dynamic Scenes](https://arxiv.org/abs/1711.11588)                          | Monocular, Stereo, RGB-D    | Focus on dynamic environments.                  | [GitHub](https://github.com/BertaBescos/DynaSLAM)                       | [Paper](https://arxiv.org/abs/1711.11588)                                   | CC BY-NC 4.0        |
| 2021 | DROID-SLAM      | [DROID-SLAM: A Video SLAM System for Dynamic Scenes](https://arxiv.org/abs/2106.13050)                                     | Monocular, Stereo, RGB-D    | Handles dynamic objects effectively.           | [GitHub](https://github.com/princeton-vl/DROID-SLAM)                    | [Paper](https://arxiv.org/abs/2106.13050)                                   | BSD-3               |
| 2021 | Cube-SLAM       | [Cube-SLAM: A Robust Visual-Inertial SLAM System for Indoor Environments](https://arxiv.org/abs/2011.00713)                | Monocular                   | Specifically for indoor use cases.             | [GitHub](https://github.com/shichaoy/cube_slam)                        | [Paper](https://arxiv.org/abs/2011.00713)                                   | BSD-3               |
| 2021 | VDO-SLAM        | [VDO-SLAM: Visual-Depth Odometry SLAM](https://arxiv.org/abs/2208.01023)                                                  | RGB-D                     | Combines visual and depth information.         | [GitHub](https://github.com/halajun/VDO_SLAM)                            | [Paper](https://arxiv.org/abs/2208.01023)                                   | GPL-3.0             |
| 2023 | Gaussian Splatting SLAM | [Gaussian Splatting SLAM: A Method for Robust RGB-D SLAM](https://arxiv.org/abs/2207.02482)                                       | RGB-D                     | Robustness in SLAM applications.               | [GitHub](https://github.com/muskie82/MonoGS)                           | [Paper](https://arxiv.org/abs/2207.02482)                                   | Not provided        |





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


- **[VINS-mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)**  
  License: GPL-3.0  
  Keywords: Feature-based, VIO, Monocular

- **[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)**  
  License: GPL-3.0  
  Keywords: Feature-based, VIO, Monocular, Stereo


  - **[OpenVINS](https://github.com/rpng/open_vins)**  
  License: GPL-3.0  
  Keywords: Feature-based, ROS1/ROS2


- **[OKVIS](https://github.com/ethz-asl/okvis)**  
  License: BSD-3  
  Keywords: Feature-based
  
- **[SVO](https://github.com/uzh-rpg/rpg_svo)**  
  License: GPL-3.0  
  Keywords: Direct-based, Monocular

- **[SVO 2.0](https://github.com/uzh-rpg/rpg_svo_pro_open)**  
  License: GPL-3.0  
  Keywords: Direct-based, Monocular, Stereo, Fish-Eye

- **[DynaSLAM](https://github.com/BertaBescos/DynaSLAM)**  
  License: CC BY-NC 4.0  
  Keywords: Monocular, Stereo, RGB-D


- **[VDO-SLAM](https://github.com/halajun/VDO_SLAM)**  
  License: GPL-3.0  
  Keywords: RGB-D, Dynamic SLAM

- **[DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM)**  
  License: BSD-3  
  Keywords: DNN-based, Monocular, Stereo, RGB-D


- **[Cube-SLAM](https://github.com/shichaoy/cube_slam)**  
  License: BSD-3  
  Keywords: Feature-based, ORB-SLAM2-based, Monocular, 2D/3D Object Detection

- **[QuadricSLAM](https://github.com/qcr/quadricslam)**  
  License: BSD-3  
  Keywords: Feature-based, ORB-SLAM2-based


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


