#include "feature_tracker.h"
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace chrono;
using namespace sensor_msgs;
using namespace message_filters;
#define SHOW_UNDISTORTION 0


// mtx lock for two threads
std::mutex mtx_lidar;

// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

// global depth register for obtaining depth of a feature
DepthRegister *depthRegister;

// feature publisher for VINS estimator
ros::Publisher pub_feature;
ros::Publisher pub_feature_1;
ros::Publisher pub_feature_2;
ros::Publisher pub_match;
ros::Publisher pub_match_1;
ros::Publisher pub_match_2;
ros::Publisher pub_restart;
ros::Publisher pub_restart_1;

// feature tracker variables
FeatureTracker trackerData[NUM_OF_CAM_ALL];

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImgSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImgSyncPolicy_3;

// random sample setting
pcl::RandomSample<PointType> down_size_random_sample_filter;

void callback_three(const sensor_msgs::ImageConstPtr &msg0, 
                    const sensor_msgs::ImageConstPtr &msg1,
                    const sensor_msgs::ImageConstPtr &msg2)
{
    double cur_img_time = msg0->header.stamp.toSec();
    // 1. whether is the first frame
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // 2. Detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    // 3. turn 8UC1 into mono8
    cv_bridge::CvImageConstPtr cv_ptr0, cv_ptr1, cv_ptr2;
    try 
    {
        cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
        cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
        cv_ptr2 = cv_bridge::toCvShare(msg2, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat show_img_0 = cv_ptr0->image;
    cv::Mat show_img_1 = cv_ptr1->image;
    cv::Mat show_img_2 = cv_ptr2->image;

    // 4. Read image and get visual features
    // TODO: put two images together so that the code can be simplified using for loop
    trackerData[0].readImage(cv_ptr0->image, cur_img_time);
    trackerData[1].readImage(cv_ptr1->image, cur_img_time);
    trackerData[2].readImage(cv_ptr2->image, cur_img_time);
    #if SHOW_UNDISTORTION
        trackerData[0].showUndistortion("undistortion_" + std::to_string(i));
    #endif

    // 5. Update features' id
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

    // 6. Package the information of all visual features
    if (PUB_THIS_FRAME)
    {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points_0(new sensor_msgs::PointCloud);
        sensor_msgs::PointCloudPtr feature_points_1(new sensor_msgs::PointCloud);
        sensor_msgs::PointCloudPtr feature_points_2(new sensor_msgs::PointCloud);

        feature_points_0->header.stamp = msg0->header.stamp;
        feature_points_0->header.frame_id = "vins_body";
        feature_points_1->header.stamp = msg1->header.stamp;
        feature_points_1->header.frame_id = "vins_body";
        feature_points_2->header.stamp = msg2->header.stamp;
        feature_points_2->header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            sensor_msgs::ChannelFloat32 id_of_point;
            sensor_msgs::ChannelFloat32 u_of_point;
            sensor_msgs::ChannelFloat32 v_of_point;
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;

            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;

            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    if(i == 0)
                        feature_points_0->points.push_back(p);
                    else if(i == 1)
                        feature_points_1->points.push_back(p);
                    else if(i == 2)
                        feature_points_2->points.push_back(p);
                    // used to identify the feature point belongs to which camera
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
            if(i == 0)
            {
                feature_points_0->channels.push_back(id_of_point);
                feature_points_0->channels.push_back(u_of_point);
                feature_points_0->channels.push_back(v_of_point);
                feature_points_0->channels.push_back(velocity_x_of_point);
                feature_points_0->channels.push_back(velocity_y_of_point);
            }
            else if(i == 1)
            {
                feature_points_1->channels.push_back(id_of_point);
                feature_points_1->channels.push_back(u_of_point);
                feature_points_1->channels.push_back(v_of_point);
                feature_points_1->channels.push_back(velocity_x_of_point);
                feature_points_1->channels.push_back(velocity_y_of_point);
            }
            else
            {
                feature_points_2->channels.push_back(id_of_point);
                feature_points_2->channels.push_back(u_of_point);
                feature_points_2->channels.push_back(v_of_point);
                feature_points_2->channels.push_back(velocity_x_of_point);
                feature_points_2->channels.push_back(velocity_y_of_point);
            }
        }

        
        // 7. get features' depths from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();
        // depth_cloud_temp in vins_world
        // feature_points->points  in camera_link
        // initilize visual features' 3d coordinates aided by lidar
        sensor_msgs::ChannelFloat32 depth_of_points_0 = depthRegister->get_depth(msg0->header.stamp, show_img_0, depth_cloud_temp, trackerData[0].m_camera, feature_points_0->points, 0);
        sensor_msgs::ChannelFloat32 depth_of_points_1 = depthRegister->get_depth(msg1->header.stamp, show_img_1, depth_cloud_temp, trackerData[1].m_camera, feature_points_1->points, 1);
        sensor_msgs::ChannelFloat32 depth_of_points_2 = depthRegister->get_depth(msg2->header.stamp, show_img_2, depth_cloud_temp, trackerData[2].m_camera, feature_points_2->points, 2);

        feature_points_0->channels.push_back(depth_of_points_0);
        feature_points_1->channels.push_back(depth_of_points_1);
        feature_points_2->channels.push_back(depth_of_points_2);

           
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
        {
            pub_feature.publish(feature_points_0);
            pub_feature_1.publish(feature_points_1);
            pub_feature_2.publish(feature_points_2);
        }
        

        // publish features in image
        if (pub_match.getNumSubscribers() != 0)
        {
            cv::Mat tmp_img_0;
            cv::Mat tmp_img_1;
            cv::Mat tmp_img_2;
            cv_ptr0 = cv_bridge::cvtColor(cv_ptr0, sensor_msgs::image_encodings::RGB8);
            cv_ptr1 = cv_bridge::cvtColor(cv_ptr1, sensor_msgs::image_encodings::RGB8);
            cv_ptr2 = cv_bridge::cvtColor(cv_ptr2, sensor_msgs::image_encodings::RGB8);
            cv::Mat stereo_img_0 = cv_ptr0->image;
            cv::Mat stereo_img_1 = cv_ptr1->image;
            cv::Mat stereo_img_2 = cv_ptr2->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                if(i == 0)
                {
                    tmp_img_0 = stereo_img_0;
                    cv::cvtColor(show_img_0, tmp_img_0, CV_GRAY2RGB);
                }
                else if(i == 1)
                {
                    tmp_img_1 = stereo_img_1;
                    cv::cvtColor(show_img_1, tmp_img_1, CV_GRAY2RGB);
                }
                else if(i == 2)
                {
                    tmp_img_2 = stereo_img_2;
                    cv::cvtColor(show_img_2, tmp_img_2, CV_GRAY2RGB);
                }

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        if(i == 0)
                        {
                            cv::circle(tmp_img_0, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                        }
                        else if(i == 1)
                        {
                            cv::circle(tmp_img_1, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                        }
                        else if(i == 2)
                        {
                            cv::circle(tmp_img_2, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                        }
                        
                    } 
                    // else 
                    // {
                    //     // depth 
                    //     if(j < depth_of_points.values.size())
                    //     {
                    //         if (depth_of_points.values[j] > 0)
                    //             cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                    //         else
                    //             cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                    //     }
                    // }
                }
            }

            pub_match.publish(cv_ptr0->toImageMsg());
            pub_match_1.publish(cv_ptr1->toImageMsg());
            pub_match_2.publish(cv_ptr2->toImageMsg());
        }
    }

}

void callback_two(const sensor_msgs::ImageConstPtr &msg0, 
                  const sensor_msgs::ImageConstPtr &msg1)
{
    double cur_img_time = msg0->header.stamp.toSec();
    // 1、Whether is the first frame
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }

    // 2、Detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }


    // 3、Turn 8UC1 into mono8
    cv_bridge::CvImageConstPtr cv_ptr0, cv_ptr1;
    try 
    {
        cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
        cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat show_img_0 = cv_ptr0->image;
    cv::Mat show_img_1 = cv_ptr1->image;

    
    // 4、Read image and get visual features
    // TODO: put two images together so that the code can be simplified using for loop
    trackerData[0].readImage(cv_ptr0->image, cur_img_time);
    trackerData[1].readImage(cv_ptr1->image, cur_img_time);
    #if SHOW_UNDISTORTION
        trackerData[0].showUndistortion("undistortion_" + std::to_string(i));
    #endif


    // 5、Update features' id
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

    // 6、Package the information of all visual features
    if (PUB_THIS_FRAME)
    {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points_0(new sensor_msgs::PointCloud);
        sensor_msgs::PointCloudPtr feature_points_1(new sensor_msgs::PointCloud);

        feature_points_0->header.stamp = msg0->header.stamp;
        feature_points_0->header.frame_id = "vins_body";
        feature_points_1->header.stamp = msg1->header.stamp;
        feature_points_1->header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            sensor_msgs::ChannelFloat32 id_of_point;
            sensor_msgs::ChannelFloat32 u_of_point;
            sensor_msgs::ChannelFloat32 v_of_point;
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;

            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;

            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    if(i == 0)
                        feature_points_0->points.push_back(p);
                    else
                        feature_points_1->points.push_back(p);
                    // used to identify the feature point belongs to which camera
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }

            if(i == 0)
            {
                feature_points_0->channels.push_back(id_of_point);
                feature_points_0->channels.push_back(u_of_point);
                feature_points_0->channels.push_back(v_of_point);
                feature_points_0->channels.push_back(velocity_x_of_point);
                feature_points_0->channels.push_back(velocity_y_of_point);
            }
            else
            {
                feature_points_1->channels.push_back(id_of_point);
                feature_points_1->channels.push_back(u_of_point);
                feature_points_1->channels.push_back(v_of_point);
                feature_points_1->channels.push_back(velocity_x_of_point);
                feature_points_1->channels.push_back(velocity_y_of_point);
            }
        }

        
        // 7、get features' depths from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();
        // depth_cloud_temp in vins_world
        // feature_points->points  in camera_link
        sensor_msgs::ChannelFloat32 depth_of_points_0 = depthRegister->get_depth(msg0->header.stamp, show_img_0, depth_cloud_temp, trackerData[0].m_camera, feature_points_0->points, 0);
        sensor_msgs::ChannelFloat32 depth_of_points_1 = depthRegister->get_depth(msg1->header.stamp, show_img_1, depth_cloud_temp, trackerData[1].m_camera, feature_points_1->points, 1);
        
        feature_points_0->channels.push_back(depth_of_points_0);
        feature_points_1->channels.push_back(depth_of_points_1);

        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
        {
            pub_feature.publish(feature_points_0);
            pub_feature_1.publish(feature_points_1);
        }
        

        // publish features in image
        if (pub_match.getNumSubscribers() != 0)
        {
            cv::Mat tmp_img_0;
            cv::Mat tmp_img_1;
            cv_ptr0 = cv_bridge::cvtColor(cv_ptr0, sensor_msgs::image_encodings::RGB8);
            cv_ptr1 = cv_bridge::cvtColor(cv_ptr1, sensor_msgs::image_encodings::RGB8);
            cv::Mat stereo_img_0 = cv_ptr0->image;
            cv::Mat stereo_img_1 = cv_ptr1->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                if(i == 0)
                {
                    tmp_img_0 = stereo_img_0;
                    cv::cvtColor(show_img_0, tmp_img_0, CV_GRAY2RGB);
                }
                else
                {
                    tmp_img_1 = stereo_img_1;
                    cv::cvtColor(show_img_1, tmp_img_1, CV_GRAY2RGB);
                }

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        if(i == 0)
                        {
                            cv::circle(tmp_img_0, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                        }
                        else
                        {
                            cv::circle(tmp_img_1, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                        }
                        
                    } 
                    // else 
                    // {
                    //     // depth 
                    //     if(j < depth_of_points.values.size())
                    //     {
                    //         if (depth_of_points.values[j] > 0)
                    //             cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                    //         else
                    //             cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                    //     }
                    // }
                }
            }

            pub_match.publish(cv_ptr0->toImageMsg());
            pub_match_1.publish(cv_ptr1->toImageMsg());
        }
    }

}



void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double cur_img_time = img_msg->header.stamp.toSec();
    // initialize the first frame
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }

    // detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;

    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_img_time);
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

        #if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistortion_" + std::to_string(i));
        #endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header.stamp = img_msg->header.stamp;
        feature_points->header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        // get feature depth from lidar point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();
        // depth_cloud_temp in vins_world
        // feature_points->points  in camera_link
        sensor_msgs::ChannelFloat32 depth_of_points = depthRegister->get_depth(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points->points, 0);
        feature_points->channels.push_back(depth_of_points);
        
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature.publish(feature_points);

        // publish features in image
        if (pub_match.getNumSubscribers() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    } else {
                        // depth 
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }

            pub_match.publish(ptr->toImageMsg());
        }
    }
}


void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;

    // 0. listen to transform
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, ros::Duration(0.01));
        listener.lookupTransform("vins_world", "vins_body_ros", laser_msg->header.stamp, transform);
    } 
    catch (tf::TransformException ex){
        // ROS_ERROR("lidar no tf");
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform.getOrigin().x();
    yCur = transform.getOrigin().y();
    zCur = transform.getOrigin().z();
    tf::Matrix3x3 m(transform.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 1. convert laser cloud message to pcl，此时laser_cloud_in是pcl格式的/hesai/pandar点云(360°的)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);



    
    // 2. downsample current cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());

    down_size_random_sample_filter.setInputCloud(laser_cloud_in);
    down_size_random_sample_filter.setSample(laser_cloud_in->size() * 0.3);
    down_size_random_sample_filter.filter(*laser_cloud_in_ds);

    // static pcl::VoxelGrid<PointType> downSizeFilter;
    // downSizeFilter.setLeafSize(0.4, 0.4, 0.4);
    // downSizeFilter.setInputCloud(laser_cloud_in);
    // downSizeFilter.filter(*laser_cloud_in_ds);

    *laser_cloud_in = *laser_cloud_in_ds;

    // // 3. filter lidar points (only keep points in camera view) 滤除相机视野外的激光点
    // pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    // Eigen::Matrix3d R_cam2lidar;
    // R_cam2lidar = Eigen::AngleAxisd(C_L_RZ, Eigen::Vector3d::UnitZ()) *
    //               Eigen::AngleAxisd(C_L_RY, Eigen::Vector3d::UnitY()) *
    //               Eigen::AngleAxisd(C_L_RX, Eigen::Vector3d::UnitX());
    // for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    // {
    //     PointType p = laser_cloud_in->points[i];
    //     if(R_cam2lidar(0,2) < 0)
    //     {
    //         if (p.x <= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
    //             laser_cloud_in_filter->push_back(p);
    //     }
    //     else
    //     {
    //         if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
    //             laser_cloud_in_filter->push_back(p);
    //     }
        
    // }
    // *laser_cloud_in = *laser_cloud_in_filter;


    // TODO: transform to IMU body frame
    // 4. offset T_lidar -> T_camera 
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    Eigen::Affine3f transOffset = pcl::getTransformation(0, 
                                                         0, 
                                                         0, 
                                                         0, 
                                                         0, 
                                                         0);  // lidar to vins_body_ros

    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 2.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        } 
        else 
        {
            break;
        }
    }

    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    down_size_random_sample_filter.setInputCloud(depthCloud);
    down_size_random_sample_filter.setSample(depthCloud->size() * 0.3);
    down_size_random_sample_filter.filter(*depthCloudDS);

    // downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    // downSizeFilter.setInputCloud(depthCloud);
    // downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Feature Tracker Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    // read extrinsic params between cam and lidar
    readParameters(n);

    ros::Subscriber sub_img;
    ros::Subscriber sub_lidar;
    message_filters::Synchronizer<ImgSyncPolicy>* Sync_;
    message_filters::Synchronizer<ImgSyncPolicy_3>* Sync_3;
    message_filters::Subscriber<sensor_msgs::Image> sub_img0;
    message_filters::Subscriber<sensor_msgs::Image> sub_img1;
    message_filters::Subscriber<sensor_msgs::Image> sub_img2;
    
    // read camera params including camera's type(pinhole et.al), camera's intrinsic and camera's name
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);
    }

    // load fisheye mask to remove features on the boundry
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_ERROR("load fisheye mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // initialize depthRegister (after readParameters())
    depthRegister = new DepthRegister(n);
    
    // subscriber to image and lidar
    if(NUM_OF_CAM == 1)
    {
        sub_img   = n.subscribe(IMAGE_TOPIC_0, 5,    img_callback);
        sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 5,    lidar_callback);

        if (!USE_LIDAR)
            sub_lidar.shutdown();

        // messages to vins estimator
        pub_feature = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature",     5);
        pub_match   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5);
        
        pub_restart = n.advertise<std_msgs::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5);
    }
    else if(NUM_OF_CAM == 2)
    {
        sub_img0.subscribe(n, IMAGE_TOPIC_0, 5);
        sub_img1.subscribe(n, IMAGE_TOPIC_1, 5);

        ImgSyncPolicy policy_scans(200);
        Sync_ = new message_filters::Synchronizer<ImgSyncPolicy>(ImgSyncPolicy(policy_scans), sub_img0, sub_img1);
        Sync_->registerCallback(boost::bind(callback_two, _1, _2));

        sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 5,    lidar_callback);
        
        if (!USE_LIDAR)
            sub_lidar.shutdown();

        // messages to vins estimator
        pub_feature = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature",     5);
        pub_feature_1 = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature_1",     5);
        pub_match   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5);
        pub_match_1   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img_1", 5);
        
        pub_restart = n.advertise<std_msgs::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5);
    }
    else if(NUM_OF_CAM == 3)
    {
        sub_img0.subscribe(n, IMAGE_TOPIC_0, 5);
        sub_img1.subscribe(n, IMAGE_TOPIC_1, 5);
        sub_img2.subscribe(n, IMAGE_TOPIC_2, 5);

        ImgSyncPolicy_3 policy_scans(200);
        Sync_3 = new message_filters::Synchronizer<ImgSyncPolicy_3>(ImgSyncPolicy_3(policy_scans), sub_img0, sub_img1, sub_img2);
        Sync_3->registerCallback(boost::bind(callback_three, _1, _2, _3));

        sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 5,    lidar_callback);
        
        if (!USE_LIDAR)
            sub_lidar.shutdown();

        // messages to vins estimator
        pub_feature = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature",     5);
        pub_feature_1 = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature_1",     5);
        pub_feature_2 = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature_2",     5);
        pub_match   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5);
        pub_match_1   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img_1", 5);
        pub_match_2   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img_2", 5);
        
        pub_restart = n.advertise<std_msgs::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5);
    }
    else
    {
        ROS_INFO("num of cam is wrong!");
        return 0;
    }
    
    // // two ROS spinners for parallel processing (image and lidar)
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}