#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/random_sample.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <cassert>
#include <glog/logging.h>

using namespace std;

typedef pcl::PointXYZI PointType;



extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
// TODO: move this to yaml
const int NUM_OF_CAM_ALL = 2;
extern int NUM_OF_CAM;
extern int num_of_cam;


extern std::string PROJECT_NAME;
extern std::string IMAGE_TOPIC_0;
extern std::string IMAGE_TOPIC_1;
extern std::string IMAGE_TOPIC_2;
extern std::string IMU_TOPIC;
extern std::string POINT_CLOUD_TOPIC;

extern int USE_LIDAR;
extern int LIDAR_SKIP;

extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

extern double C_L_TX;
extern double C_L_TY;
extern double C_L_TZ;
extern double C_L_RX;
extern double C_L_RY;
extern double C_L_RZ;

extern double C_L_TX1;
extern double C_L_TY1;
extern double C_L_TZ1;
extern double C_L_RX1;
extern double C_L_RY1;
extern double C_L_RZ1;

extern double C_L_TX2;
extern double C_L_TY2;
extern double C_L_TZ2;
extern double C_L_RX2;
extern double C_L_RY2;
extern double C_L_RZ2;

extern Eigen::Matrix3d extRot_lidar2imu;
extern Eigen::Vector3d extTrans_lidar2imu;


void readParameters(ros::NodeHandle &n);

float pointDistance(PointType p);

float pointDistance(PointType p1, PointType p2);

void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);