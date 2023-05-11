# EMV-LIO
## An Efficient Multiple Vision aided LiDAR-Inertial Odometry

**EMV-LIO** is an Efficient Multiple vision aided LiDAR-inertial odometry system based on **LVI-SAM**, which introduces multiple cameras in the VIO subsystem to expand the range of visual observation to guarantee the whole system can still maintain the relatively high accuracy in case of the failure of the monocular visual observation. Apart from this, an efficiency-enhanced LVIO system is also introduced to increase the system’s efficiency, including removing LiDAR’s noise via range image, setting condition for nearest neighbor search, and replacing kd-Tree with ikd-Tree. 

Our implementation will be available upon acceptance

<p align='center'>
    <img src="./demo.gif" alt="drawing" width="800"/>
</p>

---

## 1. Prerequisites

### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation) and its additional ROS pacakge:

### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

```
  sudo apt-get install -y libgoogle-glog-dev
  sudo apt-get install -y libatlas-base-dev
  wget -O ~/Downloads/ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip
  cd ~/Downloads/ && unzip ceres.zip -d ~/Downloads/
  cd ~/Downloads/ceres-solver-1.14.0
  mkdir ceres-bin && cd ceres-bin
  cmake ..
  sudo make install -j4
```

### 1.3. **GTSAM**

Install the dependencies
```
sudo apt-get install libboost-all-dev
sudo apt-get install cmake
sudo apt-get install libtbb-dev
```
Compile the GTSAM's code 
```
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam/
mkdir build &&cd build
cmake ..
make check 
sudo make install 
```

---

## 2. Compile
You can use the following commands to download and compile the package.

```
cd ~/emv_ws/src
git clone https://github.com/BingqiShen/EMV-LIO.git
cd ..
catkin_make -j4
```

---

## 3. Run our examples


### 3.1 Download our rosbag files

The datasets used in the paper can be downloaded from Google Drive. The data-gathering sensor suite includes: HESAI PandarXT-32 LiDAR, DAHENG MER2-202 camera, and Xsens MTi-300 IMU.
