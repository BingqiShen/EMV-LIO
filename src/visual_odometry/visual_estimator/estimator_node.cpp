#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> feature1_buf;
queue<sensor_msgs::PointCloudConstPtr> feature2_buf;

// global variable saving the lidar odometry
deque<nav_msgs::Odometry> odomQueue;
odometryRegister *odomRegister;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;
std::mutex m_odom;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_feature_1 = 0;
bool init_feature_2 = 0;
bool init_imu = 1;
double last_imu_t = 0;

// 从IMU测量值imu_msg和上一个PVQ递推得到当前PVQ.https://blog.csdn.net/weixin_42846216/article/details/106605612
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (ros::ok())
    {
        if (imu_buf.empty() || feature_buf.empty())
        {
            return measurements;
        }
        // 如果最后一个imu的时间戳都比第一个图像的时间戳小，说明图像没来，直接return掉
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            return measurements;
        }
        // 第一个imu的时间戳比第一个图像的时间戳大，那就把第一个图像pop掉
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    
    return measurements;
}

std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, 
            sensor_msgs::PointCloudConstPtr, 
            sensor_msgs::PointCloudConstPtr>>
getMeasurements_two()
{
    std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, 
                           sensor_msgs::PointCloudConstPtr, 
                           sensor_msgs::PointCloudConstPtr>> measurements;
    while (ros::ok())
    {
        if (imu_buf.empty() || feature_buf.empty() || feature1_buf.empty())
        {
            return measurements;
        }
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td || 
                imu_buf.back()->header.stamp.toSec() > feature1_buf.front()->header.stamp.toSec() + estimator.td))
        {
            return measurements;
        }
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            feature1_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        sensor_msgs::PointCloudConstPtr img1_msg = feature1_buf.front();
        feature_buf.pop();
        feature1_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg, img1_msg);
    }
    
    return measurements;
}

std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, 
            sensor_msgs::PointCloudConstPtr, 
            sensor_msgs::PointCloudConstPtr,
            sensor_msgs::PointCloudConstPtr>>
getMeasurements_three()
{
    std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, 
                           sensor_msgs::PointCloudConstPtr, 
                           sensor_msgs::PointCloudConstPtr,
                           sensor_msgs::PointCloudConstPtr>> measurements;
    while (ros::ok())
    {
        if (imu_buf.empty() || feature_buf.empty() || feature1_buf.empty() || feature2_buf.empty())
        {
            return measurements;
        }
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td || 
                imu_buf.back()->header.stamp.toSec() > feature1_buf.front()->header.stamp.toSec() + estimator.td ||
                imu_buf.back()->header.stamp.toSec() > feature2_buf.front()->header.stamp.toSec() + estimator.td))
        {
            return measurements;
        }
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            feature1_buf.pop();
            feature2_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        sensor_msgs::PointCloudConstPtr img1_msg = feature1_buf.front();
        sensor_msgs::PointCloudConstPtr img2_msg = feature2_buf.front();

        feature_buf.pop();
        feature1_buf.pop();
        feature2_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg, img1_msg, img2_msg);
    }
    
    return measurements;
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);

        std_msgs::Header header = imu_msg->header;
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header, estimator.failureCount);
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    m_odom.lock();
    odomQueue.push_back(*odom_msg);
    m_odom.unlock();
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void feature1_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature_1)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature_1 = 1;
        return;
    }
    m_buf.lock();
    feature1_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void feature2_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature_2)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature_2 = 1;
        return;
    }
    m_buf.lock();
    feature2_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}


void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// thread: visual-inertial odometry
void process()
{
    while (ros::ok())
    {
        if(NUM_OF_CAM == 1)
        {
            std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, 
                                              sensor_msgs::PointCloudConstPtr>> measurements;
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, [&]
                        {
                // align imu and image
                return (measurements = getMeasurements()).size() != 0;
                        });
            lk.unlock();

            m_estimator.lock();
            for (auto &measurement : measurements)
            {
                auto img_msg = measurement.second;

                // 1. IMU pre-integration
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                for (auto &imu_msg : measurement.first)
                {
                    double t = imu_msg->header.stamp.toSec();
                    double img_t = img_msg->header.stamp.toSec() + estimator.td;
                    if (t <= img_t)
                    { 
                        if (current_time < 0)
                            current_time = t;
                        double dt = t - current_time;
                        ROS_ASSERT(dt >= 0);
                        current_time = t;
                        dx = imu_msg->linear_acceleration.x;
                        dy = imu_msg->linear_acceleration.y;
                        dz = imu_msg->linear_acceleration.z;
                        rx = imu_msg->angular_velocity.x;
                        ry = imu_msg->angular_velocity.y;
                        rz = imu_msg->angular_velocity.z;
                        // preintegration
                        estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                    }
                    else
                    {
                        double dt_1 = img_t - current_time;
                        double dt_2 = t - img_t;
                        current_time = img_t;
                        ROS_ASSERT(dt_1 >= 0);
                        ROS_ASSERT(dt_2 >= 0);
                        ROS_ASSERT(dt_1 + dt_2 > 0);
                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);
                        dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                        dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                        dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                        rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                        ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                        rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                        estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                    }
                }

                // 2. VINS Optimization
                // TicToc t_s;
                map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;
                for (unsigned int i = 0; i < img_msg->points.size(); i++)
                {
                    int v = img_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x = img_msg->points[i].x;
                    double y = img_msg->points[i].y;
                    double z = img_msg->points[i].z;
                    double p_u = img_msg->channels[1].values[i];
                    double p_v = img_msg->channels[2].values[i];
                    double velocity_x = img_msg->channels[3].values[i];
                    double velocity_y = img_msg->channels[4].values[i];
                    double depth = img_msg->channels[5].values[i];

                    ROS_ASSERT(z == 1);
                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                    xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                    image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
                }

                // Get initialization info from lidar odometry
                vector<float> initialization_info;
                m_odom.lock();
                initialization_info = odomRegister->getOdometry(odomQueue, img_msg->header.stamp.toSec() + estimator.td, 0);
                m_odom.unlock();

                estimator.processImage(image, initialization_info, img_msg->header, 0);
                // double whole_t = t_s.toc();
                // printStatistics(estimator, whole_t);

                // 3. Visualization
                std_msgs::Header header = img_msg->header;
                pubOdometry(estimator, header);
                pubKeyPoses(estimator, header);
                pubCameraPose(estimator, header);
                pubPointCloud(estimator, header);
                pubTF(estimator, header);
                pubKeyframe(estimator);

            }

            
            m_estimator.unlock();

            m_buf.lock();
            m_state.lock();
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                update();
            m_state.unlock();
            m_buf.unlock();
        }

        else if(NUM_OF_CAM == 2)
        {
            std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, 
                                   sensor_msgs::PointCloudConstPtr,
                                   sensor_msgs::PointCloudConstPtr>> measurements;
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, [&]
                    {
                return (measurements = getMeasurements_two()).size() != 0;
                    });
            lk.unlock();

            m_estimator.lock();
            for (auto &measurement : measurements)
            {
                auto img_msg = std::get<1>(measurement);
                auto img1_msg = std::get<2>(measurement);

                // 1. IMU pre-integration
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                for (auto &imu_msg : std::get<0>(measurement))
                {
                    double t = imu_msg->header.stamp.toSec();
                    double img_t = img_msg->header.stamp.toSec() + estimator.td;
                    if (t <= img_t)
                    {
                        if (current_time < 0)
                            current_time = t;
                        double dt = t - current_time;
                        ROS_ASSERT(dt >= 0);
                        current_time = t;
                        dx = imu_msg->linear_acceleration.x;
                        dy = imu_msg->linear_acceleration.y;
                        dz = imu_msg->linear_acceleration.z;
                        rx = imu_msg->angular_velocity.x;
                        ry = imu_msg->angular_velocity.y;
                        rz = imu_msg->angular_velocity.z;
                        estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                    }
                    else
                    {
                        double dt_1 = img_t - current_time;
                        double dt_2 = t - img_t;
                        current_time = img_t;
                        ROS_ASSERT(dt_1 >= 0);
                        ROS_ASSERT(dt_2 >= 0);
                        ROS_ASSERT(dt_1 + dt_2 > 0);
                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);
                        dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                        dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                        dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                        rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                        ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                        rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                        estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                    }
                }

                // 2. VINS Optimization
                // TicToc t_s;
                map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;
                for (unsigned int i = 0; i < img_msg->points.size(); i++)
                {
                    int v = img_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x = img_msg->points[i].x;
                    double y = img_msg->points[i].y;
                    double z = img_msg->points[i].z;
                    double p_u = img_msg->channels[1].values[i];
                    double p_v = img_msg->channels[2].values[i];
                    double velocity_x = img_msg->channels[3].values[i];
                    double velocity_y = img_msg->channels[4].values[i];
                    double depth = img_msg->channels[5].values[i];

                    ROS_ASSERT(z == 1);
                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                    xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                    image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
                }
                map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image1;
                for (unsigned int i = 0; i < img1_msg->points.size(); i++)
                {
                    int v1 = img1_msg->channels[0].values[i] + 0.5;
                    int feature1_id = v1 / NUM_OF_CAM;
                    int camera1_id = v1 % NUM_OF_CAM;
                    double x1 = img1_msg->points[i].x;
                    double y1 = img1_msg->points[i].y;
                    double z1 = img1_msg->points[i].z;
                    double p_u1 = img1_msg->channels[1].values[i];
                    double p_v1 = img1_msg->channels[2].values[i];
                    double velocity_x1 = img1_msg->channels[3].values[i];
                    double velocity_y1 = img1_msg->channels[4].values[i];
                    double depth1 = img1_msg->channels[5].values[i];

                    ROS_ASSERT(z1 == 1);
                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth1;
                    xyz_uv_velocity_depth1 << x1, y1, z1, p_u1, p_v1, velocity_x1, velocity_y1, depth1;
                    image1[feature1_id].emplace_back(camera1_id,  xyz_uv_velocity_depth1);
                }

                // Get initialization info from lidar odometry
                vector<float> initialization_info, initialization_info_1;
                m_odom.lock();
                initialization_info = odomRegister->getOdometry(odomQueue, img_msg->header.stamp.toSec() + estimator.td, 0);
                initialization_info_1 = odomRegister->getOdometry(odomQueue, img1_msg->header.stamp.toSec() + estimator.td, 1);

                m_odom.unlock();
                estimator.processImage2(image, image1, initialization_info, initialization_info_1, img_msg->header);

                // double whole_t = t_s.toc();
                // printStatistics(estimator, whole_t);

                // 3. Visualization
                std_msgs::Header header = img_msg->header;
                pubOdometry(estimator, header);
                pubKeyPoses(estimator, header);

                pubCameraPose(estimator, header);
                pubCameraPose1(estimator, header);
                
                pubPointCloud(estimator, header);
                pubPointCloud1(estimator, header);

                pubTF(estimator, header);
                pubKeyframe(estimator);

            }

            
            m_estimator.unlock();

            m_buf.lock();
            m_state.lock();
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                update();
            m_state.unlock();
            m_buf.unlock();
        }

        else if(NUM_OF_CAM == 3)
        {
            std::vector<std::tuple<std::vector<sensor_msgs::ImuConstPtr>, 
                                   sensor_msgs::PointCloudConstPtr,
                                   sensor_msgs::PointCloudConstPtr,
                                   sensor_msgs::PointCloudConstPtr>> measurements;
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, [&]
                    {
                // align imu and image
                return (measurements = getMeasurements_three()).size() != 0;
                    });
            lk.unlock();

            m_estimator.lock();
            for (auto &measurement : measurements)
            {
                auto img_msg = std::get<1>(measurement);
                auto img1_msg = std::get<2>(measurement);
                auto img2_msg = std::get<3>(measurement);

                // 1. IMU pre-integration
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                for (auto &imu_msg : std::get<0>(measurement))
                {
                    double t = imu_msg->header.stamp.toSec();
                    double img_t = img_msg->header.stamp.toSec() + estimator.td;
                    if (t <= img_t)
                    {
                        if (current_time < 0)
                            current_time = t;
                        double dt = t - current_time;
                        ROS_ASSERT(dt >= 0);
                        current_time = t;
                        dx = imu_msg->linear_acceleration.x;
                        dy = imu_msg->linear_acceleration.y;
                        dz = imu_msg->linear_acceleration.z;
                        rx = imu_msg->angular_velocity.x;
                        ry = imu_msg->angular_velocity.y;
                        rz = imu_msg->angular_velocity.z;
                        // IMU preintegration
                        estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                    }
                    else
                    {
                        double dt_1 = img_t - current_time;
                        double dt_2 = t - img_t;
                        current_time = img_t;
                        ROS_ASSERT(dt_1 >= 0);
                        ROS_ASSERT(dt_2 >= 0);
                        ROS_ASSERT(dt_1 + dt_2 > 0);
                        double w1 = dt_2 / (dt_1 + dt_2);
                        double w2 = dt_1 / (dt_1 + dt_2);
                        dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                        dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                        dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                        rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                        ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                        rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                        estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                        //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                    }
                }

                // 2. VINS Optimization
                // TicToc t_s;
                map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;
                for (unsigned int i = 0; i < img_msg->points.size(); i++)
                {
                    int v = img_msg->channels[0].values[i] + 0.5;
                    int feature_id = v / NUM_OF_CAM;
                    int camera_id = v % NUM_OF_CAM;
                    double x = img_msg->points[i].x;
                    double y = img_msg->points[i].y;
                    double z = img_msg->points[i].z;
                    double p_u = img_msg->channels[1].values[i];
                    double p_v = img_msg->channels[2].values[i];
                    double velocity_x = img_msg->channels[3].values[i];
                    double velocity_y = img_msg->channels[4].values[i];
                    double depth = img_msg->channels[5].values[i];

                    ROS_ASSERT(z == 1);
                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                    xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                    image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
                }
                map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image1;
                for (unsigned int i = 0; i < img1_msg->points.size(); i++)
                {
                    int v1 = img1_msg->channels[0].values[i] + 0.5;
                    int feature1_id = v1 / NUM_OF_CAM;
                    int camera1_id = v1 % NUM_OF_CAM;
                    double x1 = img1_msg->points[i].x;
                    double y1 = img1_msg->points[i].y;
                    double z1 = img1_msg->points[i].z;
                    double p_u1 = img1_msg->channels[1].values[i];
                    double p_v1 = img1_msg->channels[2].values[i];
                    double velocity_x1 = img1_msg->channels[3].values[i];
                    double velocity_y1 = img1_msg->channels[4].values[i];
                    double depth1 = img1_msg->channels[5].values[i];

                    ROS_ASSERT(z1 == 1);
                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth1;
                    xyz_uv_velocity_depth1 << x1, y1, z1, p_u1, p_v1, velocity_x1, velocity_y1, depth1;
                    image1[feature1_id].emplace_back(camera1_id,  xyz_uv_velocity_depth1);
                }
                map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image2;
                for (unsigned int i = 0; i < img2_msg->points.size(); i++)
                {
                    int v2 = img2_msg->channels[0].values[i] + 0.5;
                    int feature2_id = v2 / NUM_OF_CAM;
                    int camera2_id = v2 % NUM_OF_CAM;
                    double x2 = img2_msg->points[i].x;
                    double y2 = img2_msg->points[i].y;
                    double z2 = img2_msg->points[i].z;
                    double p_u2 = img2_msg->channels[1].values[i];
                    double p_v2 = img2_msg->channels[2].values[i];
                    double velocity_x2 = img2_msg->channels[3].values[i];
                    double velocity_y2 = img2_msg->channels[4].values[i];
                    double depth2 = img2_msg->channels[5].values[i];

                    ROS_ASSERT(z2 == 1);
                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth2;
                    xyz_uv_velocity_depth2 << x2, y2, z2, p_u2, p_v2, velocity_x2, velocity_y2, depth2;
                    image2[feature2_id].emplace_back(camera2_id,  xyz_uv_velocity_depth2);
                }

                // Get initialization info from lidar odometry
                vector<float> initialization_info, initialization_info_1, initialization_info_2;
                m_odom.lock();
                initialization_info = odomRegister->getOdometry(odomQueue, img_msg->header.stamp.toSec() + estimator.td, 0);
                initialization_info_1 = odomRegister->getOdometry(odomQueue, img1_msg->header.stamp.toSec() + estimator.td, 1);
                initialization_info_2 = odomRegister->getOdometry(odomQueue, img2_msg->header.stamp.toSec() + estimator.td, 2);

                m_odom.unlock();
                estimator.processImage3(image, image1, image2, 
                                        initialization_info, initialization_info_1, initialization_info_2, img_msg->header);

                // double whole_t = t_s.toc();
                // printStatistics(estimator, whole_t);

                // 3. Visualization
                std_msgs::Header header = img_msg->header;
                pubOdometry(estimator, header);
                pubKeyPoses(estimator, header);

                pubCameraPose(estimator, header);
                pubCameraPose1(estimator, header);
                pubCameraPose2(estimator, header);
                
                pubPointCloud(estimator, header);
                pubPointCloud1(estimator, header);
                pubPointCloud2(estimator, header);

                pubTF(estimator, header);
                pubKeyframe(estimator);

            }

            
            m_estimator.unlock();

            m_buf.lock();
            m_state.lock();
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                update();
            m_state.unlock();
            m_buf.unlock();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Odometry Estimator Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    // read parameters including extrinsic between cam and lidar and imu
    readParameters(n);
    // set the extrinsic params between cam and imu
    estimator.setParameter();

    registerPub(n);

    odomRegister = new odometryRegister(n);
    
    ros::Subscriber sub_image;
    ros::Subscriber sub_image1;
    ros::Subscriber sub_image2;
    ros::Subscriber sub_restart;

    ros::Subscriber sub_imu     = n.subscribe(IMU_TOPIC,      5000, imu_callback,  ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_odom    = n.subscribe("odometry/imu", 5000, odom_callback);
    if(NUM_OF_CAM == 1)
    {
        sub_image   = n.subscribe(PROJECT_NAME + "/vins/feature/feature", 1, feature_callback);
        sub_restart = n.subscribe(PROJECT_NAME + "/vins/feature/restart", 1, restart_callback);
    }
    else if(NUM_OF_CAM == 2)
    {
        sub_image   = n.subscribe(PROJECT_NAME + "/vins/feature/feature", 1, feature_callback);
        sub_image1  = n.subscribe(PROJECT_NAME + "/vins/feature/feature_1", 1, feature1_callback);
        sub_restart = n.subscribe(PROJECT_NAME + "/vins/feature/restart", 1, restart_callback);
    }
    else if(NUM_OF_CAM == 3)
    {
        sub_image   = n.subscribe(PROJECT_NAME + "/vins/feature/feature", 1, feature_callback);
        sub_image1  = n.subscribe(PROJECT_NAME + "/vins/feature/feature_1", 1, feature1_callback);
        sub_image2  = n.subscribe(PROJECT_NAME + "/vins/feature/feature_2", 1, feature2_callback);
        sub_restart = n.subscribe(PROJECT_NAME + "/vins/feature/restart", 1, restart_callback);
    }
    
    if (!USE_LIDAR)
        sub_odom.shutdown();

    std::thread measurement_process{process};

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}