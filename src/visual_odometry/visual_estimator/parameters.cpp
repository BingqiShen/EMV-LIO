#include "parameters.h"
#include <vector>

std::string PROJECT_NAME;

double INIT_DEPTH;
double MIN_PARALLAX;
double IMU_FREQ;
double DELTA_T_IMU;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

int USE_LIDAR;
int ALIGN_CAMERA_LIDAR_COORDINATE;

int NUM_OF_CAM;

double C_L_TX;
double C_L_TY;
double C_L_TZ;
double C_L_RX;
double C_L_RY;
double C_L_RZ;

double C_L_TX1;
double C_L_TY1;
double C_L_TZ1;
double C_L_RX1;
double C_L_RY1;
double C_L_RZ1;

double C_L_TX2;
double C_L_TY2;
double C_L_TZ2;
double C_L_RX2;
double C_L_RY2;
double C_L_RZ2;

std::vector<double> extRotV_lidar2imu;
std::vector<double> extTransV_lidar2imu;
Eigen::Matrix3d extRot_lidar2imu;
Eigen::Vector3d extTrans_lidar2imu;

void readParameters(ros::NodeHandle &n)
{
    // lidar parameters by sbq
    n.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "sam");
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicRot", extRotV_lidar2imu, std::vector<double>());
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicTrans", extTransV_lidar2imu, std::vector<double>());
    extRot_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV_lidar2imu.data(), 3, 3);
    extTrans_lidar2imu = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV_lidar2imu.data(), 3, 1);

    n.param<int>(PROJECT_NAME+ "/NUM_OF_CAM", NUM_OF_CAM, 1);

    std::string config_file, config_file1, config_file2;
    n.getParam("vins_config_file", config_file);
    n.getParam("vins_config_file_1", config_file1);
    n.getParam("vins_config_file_2", config_file2);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    cv::FileStorage fsSettings1(config_file1, cv::FileStorage::READ);
    cv::FileStorage fsSettings2(config_file2, cv::FileStorage::READ);

    if(!fsSettings.isOpened() || !fsSettings1.isOpened() || !fsSettings2.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["project_name"] >> PROJECT_NAME;
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    fsSettings["imu_topic"] >> IMU_TOPIC;

    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["align_camera_lidar_estimation"] >> ALIGN_CAMERA_LIDAR_COORDINATE;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;


    C_L_TX = fsSettings["cam_to_lidar_tx"];
    C_L_TY = fsSettings["cam_to_lidar_ty"];
    C_L_TZ = fsSettings["cam_to_lidar_tz"];
    C_L_RX = fsSettings["cam_to_lidar_rx"];
    C_L_RY = fsSettings["cam_to_lidar_ry"];
    C_L_RZ = fsSettings["cam_to_lidar_rz"];

    C_L_TX1 = fsSettings1["cam_to_lidar_tx"];
    C_L_TY1 = fsSettings1["cam_to_lidar_ty"];
    C_L_TZ1 = fsSettings1["cam_to_lidar_tz"];
    C_L_RX1 = fsSettings1["cam_to_lidar_rx"];
    C_L_RY1 = fsSettings1["cam_to_lidar_ry"];
    C_L_RZ1 = fsSettings1["cam_to_lidar_rz"];

    C_L_TX2 = fsSettings2["cam_to_lidar_tx"];
    C_L_TY2 = fsSettings2["cam_to_lidar_ty"];
    C_L_TZ2 = fsSettings2["cam_to_lidar_tz"];
    C_L_RX2 = fsSettings2["cam_to_lidar_rx"];
    C_L_RY2 = fsSettings2["cam_to_lidar_ry"];
    C_L_RZ2 = fsSettings2["cam_to_lidar_rz"];

    IMU_FREQ = fsSettings["imu_freq"];
    DELTA_T_IMU = double(1/IMU_FREQ);
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("Image dimention: ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_INFO(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = pkg_path + "/config/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_INFO(" Fix extrinsic param.");

        cv::Mat cv_R, cv_T, cv_R1, cv_T1, cv_R2, cv_T2;
        fsSettings["extrinsicRotation"] >> cv_R; fsSettings1["extrinsicRotation"] >> cv_R1; fsSettings2["extrinsicRotation"] >> cv_R2;
        fsSettings["extrinsicTranslation"] >> cv_T; fsSettings1["extrinsicTranslation"] >> cv_T1; fsSettings2["extrinsicTranslation"] >> cv_T2;
        Eigen::Matrix3d eigen_R, eigen_R1, eigen_R2;
        Eigen::Vector3d eigen_T, eigen_T1, eigen_T2;
        cv::cv2eigen(cv_R, eigen_R); cv::cv2eigen(cv_R1, eigen_R1); cv::cv2eigen(cv_R2, eigen_R2);
        cv::cv2eigen(cv_T, eigen_T); cv::cv2eigen(cv_T1, eigen_T1); cv::cv2eigen(cv_T2, eigen_T2);
        Eigen::Quaterniond Q(eigen_R); Eigen::Quaterniond Q1(eigen_R1); Eigen::Quaterniond Q2(eigen_R2);
        eigen_R = Q.normalized(); eigen_R1 = Q1.normalized(); eigen_R2 = Q2.normalized();
        RIC.push_back(eigen_R); RIC.push_back(eigen_R1); RIC.push_back(eigen_R2);
        TIC.push_back(eigen_T); TIC.push_back(eigen_T1); TIC.push_back(eigen_T2);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }
    
    fsSettings.release();
    fsSettings1.release();
    fsSettings2.release();
    usleep(100);
}
