# imu_gnss_eskf

The project is to implement an ESKF algorithm to fuse IMU and GNSS data. The theory can be referred to [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/pdf/1711.02508.pdf). The implementation can be referred to [imu_gps_localization](https://github.com/ydsf16/imu_gps_localization). The test dataset can be referred to [EU](https://epan-utbm.github.io/utbm_robocar_dataset/).

![image](https://github.com/yanjingang/imu_gnss_eskf/blob/master/doc/imu_gnss_eskf.png)

### 1. Requirements

It is tested under Ubuntu 20.04 + ROS noetic.
* nav_msgs is used for ROS publishing. 
* eigen_conversions is used for ROS publishing.
* nmea_navsat_driver is used for GNSS data processing.
* Eigen is used for matrix computation.
* GeographicLib is used for transformation between LLA and ENU.
 ````
  sudo apt-get install libeigen3-dev 
  sudo apt-get install ros-noetic-geographic-* geographiclib-* libgeographic-*
  sudo apt-get install ros-noetic-nav-msgs ros-noetic-eigen-conversions ros-noetic-nmea-navsat-driver
  sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/
````


### 2. Build

Clone the repository
````
git clone https://github.com/yanjingang/imu_gnss_eskf.git
````
Compile
````
cd ~/catkin_ws
catkin_make
````
### 3. Run

Run launch file
````
roslaunch imu_gnss_eskf imu_gnss_eskf.launch
````
Play autoware bag file
````
rosbag play autoware-20241124082629.bag -r1 /vehicle/odom:=/vehicle/odom_discard
````





