# Super Odometry

Super Odometry is a high-performance odometry library designed for robotics applications. It integrates advanced algorithms for real-time localization and mapping, supporting a wide range of sensors.

## Merge Plan

- [x] CODE 1: Merge Sara code with MMPUG Code    (ARL are same with Darpa people are same page) (Shibo Zhao)
   - [x] Support Visual, Thermal Odometry Fusion
   - [x] Support Better Mapping Capability
   - [ ] Support Multi Robot SLAM Systems  
- [ ] CODE 2: Learning Physics code with Racer Code (Parv Maheshwari)
   - [ ] Support Multi Lidar and GPS
   - [ ] Better Comment for each function
- [ ] Big Task: Merge Code 1 (Pro: advance features  Cons: no comments ) and Code 2 (Pro: standard Cons: less advanced features)      (Parv Maheshwari, Shibo)
   - [ ] Code Format Changes : Namespace, function names....
   - [ ] Prefer to use Code 1 as code base to merge the features from Code2
- Support ROS 2.0 (Parv Maheshwari) 

## Table of Contents

- [Prerequisites](#prerequisites)
- [Docker](#docker)
- [Installation](#installation)
- [Building the Project](#building-the-project)
- [Library Overview](#library-overview)
- [Running the Project](#running-the-project)
- [System Main Topics](#system-main-topics)
- [Verification](#verification)
- [Calibration&Timesync](#calibration)
- [Contact](#contact)
- [License](#license)

## Docker 

You don't need to install follow dependency, if you have docker access. Please use 
this docker environemnts in our docker directory. 

```bash
   sh docker_build.sh ---> install the docker image
   sh docker_run.sh ---> run the system
```


## Prerequisites

Before installing Super Odometry, ensure your system meets the following requirements:
If you have any issues, please check the installation.md and update [installation.md](./installation.md). 
- **Ubuntu**: 18.04, 20.04
- **ROS**: Full installation of ROS Melodic
- **Ceres Solver**: Follow the [Ceres Installation Guide](http://ceres-solver.org/installation.html).

- **Sophus**: Clone and checkout a specific commit and find solution if face building error in [installation.md](./installation.md)

  ```bash
   git clone http://github.com/strasdat/Sophus.git
   git checkout a621ff
  ```
- **OpenCV**: Version 4.0 is required. (If you want to use visual odometry)
- **GTSAM 4.0**: Georgia Tech Smoothing and Mapping library.

   ```bash
   wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
   cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
   cd ~/Downloads/gtsam-4.0.2/
   mkdir build && cd build
   cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
   sudo make install -j8
   ```

- **libparmetis-dev**: Recommended for improved performance.
   ```bash
   sudo apt-get install libparmetis-dev
   ```

## Build Project 

Create a clean workspace and build the project using:
 
 ```bash 
   catkin_make
   # or
   catkin build
 ```

## Running the Code

-  MMPUG Project 
   - roslaunch super_odometry mmpug_laser_visual.launch (launch visual odometry)
   - roslaunch super_odometry ugv_offline.launch 
-  SARA Project 
   - roslaunch super_odometry wanda.launch        

## System Main Topics

Please look at the launch file. You can change the topic name through launch config file. 
You don't need to prvoide all three topics. If you provide only provide image and imu topic. The vio program will run. However, if you provide velodyne and imu topic. The lio program will run. However, it is better that you can provide all of them. 

The Default Input Topics:
+ Image Topic:      /camera/image_raw  (optional) 
+ Imu   Topic:      /imu/data/  
+ Veloedyne Topic:   /velodyne_points 

Output Topics:
+ Odometry Topic:    /integrated_to_init (Super Odometry)
+ Current map cloud Topic:    /velodyne_cloud_registered_imu 
+ Laser Odometry Topic: /aft_mapped_to_init

## How to verify the result
if you result is similar to [this video](https://youtu.be/ZIvkt2uCB_w), which means that you are good. If not, please contact me.

## Contact

Since the superodometry is still under construction, the code will be restricted for a while. 

Please help me to keep it confidential.I will open sourced the code when I finished it. Thank you very much!!

If you have any questions, please contact shiboz@andrew.cmu.edu

## Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

## Major Contributor 
Shibo Zhao, Parv Maheshwari, YaoYu Hu, YuanJun Gao