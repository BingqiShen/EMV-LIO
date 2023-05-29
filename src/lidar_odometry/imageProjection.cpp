#include "utility.h"
#include "emv_lio/cloud_info.h"

// Velodyne
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Ouster
struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)


// hesai(pandar)
struct HesaiPointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    HesaiPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))

// use the velodyne point format as the common representation
using PointXYZIRT = VelodynePointXYZIRT;



// for removing noist points
class ScanInfo
{
public:
    ScanInfo(const int &n_scan, const bool &segment_flag)
    {
        segment_flag_ = segment_flag;
        scan_start_ind_.resize(n_scan);
        scan_end_ind_.resize(n_scan);
        ground_flag_.clear();
    }

    std::vector<int> scan_start_ind_, scan_end_ind_;
    bool segment_flag_;
    std::vector<bool> ground_flag_;
};


const int queueLength = 1000;

std::shared_ptr<ImuTracker> imu_tracker_;
bool initialised_ = false;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;
    
    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<HesaiPointXYZIRT>::Ptr tmpHesaiCloudIn;
    pcl::PointCloud<VelodynePointXYZIRT>::Ptr tmpVelodyneCloudIn;


    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    emv_lio::cloud_info cloudInfo;
    
    double timeScanCur;
    double timeScanNext;
    std_msgs::Header cloudHeader;

    Eigen::MatrixXf range_mat;


public:
    ImageProjection():
    deskewFlag(0)
    {
        subImu        = nh.subscribe<sensor_msgs::Imu>        (imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>      (PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> (PROJECT_NAME + "/lidar/deskew/cloud_deskewed", 5);
        pubLaserCloudInfo = nh.advertise<emv_lio::cloud_info>      (PROJECT_NAME + "/lidar/deskew/cloud_info", 5);

        // initialization
        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }


    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpHesaiCloudIn.reset(new pcl::PointCloud<HesaiPointXYZIRT>());
        tmpVelodyneCloudIn.reset(new pcl::PointCloud<VelodynePointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }


    void resetParameters()
    {
        laserCloudIn->clear();
        tmpOusterCloudIn->clear();
        tmpHesaiCloudIn->clear();
        tmpVelodyneCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        range_mat = Eigen::MatrixXf::Constant(N_SCAN, Horizon_SCAN, FLT_MAX);

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection(){}

    /**
     * subscribe raw imu data
     * and convert it to the lidar coordinate
    */
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {


        if (!initialised_)
        {
            initialised_ = true;
            imu_tracker_.reset(new ImuTracker(10.0, imuMsg->header.stamp.toSec()));
        }
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg, imu_tracker_);
        
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    /**
     * subscribe imu odometry integrated by imuPreintegration
    */
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }


    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg, laserCloudIn))
            return;

        if (!deskewInfo())
            return;

        if (!remove_noise)
            projectPointCloud();
        else
        {
            ScanInfo scan_info(N_SCAN, 1);
            segmentCloud(laserCloudIn, scan_info);

            for (int i = 0; i < N_SCAN; ++i)
            {
                for (int j = 0; j < Horizon_SCAN; ++j)
                {
                    rangeMat.at<float>(i, j) = range_mat(i, j);
                }
            }
        }

        // extract valid point and save as extractedCloud
        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg,
                         pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);

        if (cloudQueue.size() <= 2)
            return false;

        currentCloudMsg = cloudQueue.front();
        cloudQueue.pop_front();

        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanNext = cloudQueue.front().header.stamp.toSec();
        
        // HESAI
        if (lidar_type == 2)
        {
            // convert to hesai format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpHesaiCloudIn);

            laserCloudIn->points.resize(tmpHesaiCloudIn->size());
            laserCloudIn->is_dense = tmpHesaiCloudIn->is_dense;
            for (size_t i = 0; i < tmpHesaiCloudIn->size(); i++)
            {
                auto& src = tmpHesaiCloudIn->points[i];
                auto& dst = laserCloudIn->points[i];

                int rowIdn = src.ring;
                if (rowIdn < 0 || rowIdn >= N_SCAN)
                    continue;
                if (rowIdn % downsampleRate != 0)
                    continue;

                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                float det_cur_time = src.timestamp - tmpHesaiCloudIn->points.front().timestamp;

                dst.time = det_cur_time;
                // points_times.insert(dst.time);
            }
        }
        // OUSTER
        else if (lidar_type == 1)
        {
            // convert to ouster format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);

            laserCloudIn->points.reserve(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto& src = tmpOusterCloudIn->points[i];
                auto& dst = laserCloudIn->points[i];

                int rowIdn = src.ring;
                if (rowIdn < 0 || rowIdn >= N_SCAN)
                    continue;
                if (rowIdn % downsampleRate != 0)
                    continue;

                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;

                float det_cur_time = src.t * 1e-9;
                dst.time = det_cur_time;

                laserCloudIn->push_back(dst);
                // points_times.insert(dst.time);
            }

            laserCloudIn->resize(laserCloudIn->size());
        }
        // VELODYNE
        else if (lidar_type == 3)
        {
            // convert to velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpVelodyneCloudIn);

            laserCloudIn->points.resize(tmpVelodyneCloudIn->size());
            laserCloudIn->is_dense = tmpVelodyneCloudIn->is_dense;
            for (size_t i = 0; i < tmpVelodyneCloudIn->size(); i++)
            {
                auto& src = tmpVelodyneCloudIn->points[i];
                auto& dst = laserCloudIn->points[i];

                int rowIdn = src.ring;
                if (rowIdn < 0 || rowIdn >= N_SCAN)
                    continue;
                if (rowIdn % downsampleRate != 0)
                    continue;

                    
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                float det_cur_time = src.time - tmpVelodyneCloudIn->points.front().time;

                dst.time = det_cur_time;
                // points_times.insert(dst.time);
            }
        }


        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }     

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == timeField)
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }


    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanNext)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanNext + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.odomX = startOdomMsg.pose.pose.position.x;
        cloudInfo.odomY = startOdomMsg.pose.pose.position.y;
        cloudInfo.odomZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.odomRoll  = roll;
        cloudInfo.odomPitch = pitch;
        cloudInfo.odomYaw   = yaw;
        cloudInfo.odomResetId = (int)round(startOdomMsg.pose.covariance[0]);

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanNext)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanNext)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanNext - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }


    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }


    void projectPointCloud()
    {
        int cloudSize = (int)laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            static float ang_res_x = 360.0/float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            float range = pointDistance(thisPoint);
            
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;    

            // common format
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); 
            
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }


    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }
    
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");
        pubLaserCloudInfo.publish(cloudInfo);
    }
    

    void segmentCloud(const pcl::PointCloud<PointXYZIRT>::Ptr &laser_cloud_in,
                            ScanInfo &scan_info)
    {
        std::vector<pair<int8_t, int8_t> > neighbor_iterator_;
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = 1;
        neighbor.second = 0;
        neighbor_iterator_.push_back(neighbor);
        neighbor.first = -1;
        neighbor.second = 0;
        neighbor_iterator_.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = 1;
        neighbor_iterator_.push_back(neighbor);
        neighbor.first = 0;
        neighbor.second = -1;
        neighbor_iterator_.push_back(neighbor);
    

        float ang_res_x = 360.0 / float(Horizon_SCAN);
        float segment_alphax, segment_alphay;

        // set specific parameters
        range_mat = Eigen::MatrixXf::Constant(N_SCAN, Horizon_SCAN, FLT_MAX);
        Eigen::MatrixXi label_mat = Eigen::MatrixXi::Zero(N_SCAN, Horizon_SCAN);

        pcl::PointCloud<PointType> cloud_matrix;
        cloud_matrix.resize(N_SCAN * Horizon_SCAN);
        std::vector<pcl::PointCloud<PointType>> cloud_scan(N_SCAN);
        std::vector<int> cloud_scan_order(N_SCAN * Horizon_SCAN);

        projectCloud(laser_cloud_in, cloud_matrix, range_mat, cloud_scan, cloud_scan_order);

        std::vector<uint16_t> all_pushed_indx(N_SCAN * Horizon_SCAN);
        std::vector<uint16_t> all_pushed_indy(N_SCAN * Horizon_SCAN);

        std::vector<uint16_t> queue_indx(N_SCAN * Horizon_SCAN);
        std::vector<uint16_t> queue_indy(N_SCAN * Horizon_SCAN);

        std::vector<int> queue_indx_last_negi(N_SCAN * Horizon_SCAN);
        std::vector<int> queue_indy_last_negi(N_SCAN * Horizon_SCAN);
        std::vector<float> queue_last_dis(N_SCAN * Horizon_SCAN);

        // remote FLT_MAX points
        for (size_t i = 0; i < N_SCAN; i++)
            for (size_t j = 0; j < Horizon_SCAN; j++)
                if (range_mat(i, j) == FLT_MAX) 
                    label_mat(i, j) = -1;

        // label ground points
        int label_count = 1;
        size_t lower_ind, upper_ind;
        float vertical_angle;
        float diff_x, diff_y, diff_z;


        for (size_t i = 0; i < N_SCAN; i++)
        {
            for (size_t j = 0; j < Horizon_SCAN; j++)
            {
                if(i != (N_SCAN-1))
                {
                    if (range_mat(i, j) == FLT_MAX || range_mat(i + 1, j) == FLT_MAX)
                        continue;
                    
                    
                    lower_ind = j + i * Horizon_SCAN;
                    upper_ind = j + (i + 1) * Horizon_SCAN;
                    const PointType &point1 = cloud_matrix.points[lower_ind];
                    const PointType &point2 = cloud_matrix.points[upper_ind];
                    diff_x = point1.x - point2.x;
                    diff_y = point1.y - point2.y;
                    diff_z = point1.z - point2.z;
                    vertical_angle = atan2(diff_z, sqrt(diff_x * diff_x + diff_y * diff_y)) * 180 / M_PI;
                    if (abs(vertical_angle) <= (10 / downsampleRate)) // 10deg
                    {
                        label_mat(i, j) = label_count;
                        label_mat(i + 1, j) = label_count;
                    }
                }
            }
        }



        label_count++;


        // BFS to search nearest neighbors
        for (size_t i = 0; i < N_SCAN; i++)
        {
            for (size_t j = 0; j < Horizon_SCAN; j++)
            {
                if (label_mat(i, j) == 0)
                {
                    int row = i;
                    int col = j;

                    float d1, d2, alpha, angle, dist;
                    int from_indx, from_indy, this_indx, this_indy;
                    bool line_count_flag[N_SCAN] = {false};

                    queue_indx[0] = row;
                    queue_indy[0] = col;
                    queue_indx_last_negi[0] = 0;
                    queue_indy_last_negi[0] = 0;
                    queue_last_dis[0] = 0;
                    int queue_size = 1;
                    int queue_start_ind = 0;
                    int queue_end_ind = 1;

                    all_pushed_indx[0] = row;
                    all_pushed_indy[0] = col;
                    int all_pushed_ind_size = 1;

                    // find the neighbor connecting clusters in range image, bfs
                    while (queue_size > 0)
                    {
                        from_indx = queue_indx[queue_start_ind];
                        from_indy = queue_indy[queue_start_ind];
                        --queue_size;
                        ++queue_start_ind;
                        label_mat(from_indx, from_indy) = label_count;
                        
                        for (auto iter = neighbor_iterator_.begin(); iter != neighbor_iterator_.end(); ++iter)
                        {
                            this_indx = from_indx + iter->first;
                            this_indy = from_indy + iter->second;

                            if (this_indx < 0 || this_indx >= N_SCAN)
                                continue;

                            segment_alphax = ang_res_x / 180.0 * M_PI;
                            segment_alphay = ang_res_y / 180.0 * M_PI;

                            if (this_indy < 0)
                                this_indy = Horizon_SCAN - 1;
                            if (this_indy >= Horizon_SCAN)
                                this_indy = 0;
                            if (label_mat(this_indx, this_indy) != 0)
                                continue;

                            d1 = std::max(range_mat(from_indx, from_indy),
                                        range_mat(this_indx, this_indy));
                            d2 = std::min(range_mat(from_indx, from_indy),
                                        range_mat(this_indx, this_indy));
                            dist = sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * cos(alpha));
                            alpha = iter->first == 0 ? segment_alphax : segment_alphay;
                            angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));
                            
                            double SEGMENT_THETA = 0.53;
                            if (angle > SEGMENT_THETA)
                            {
                                queue_indx[queue_end_ind] = this_indx;
                                queue_indy[queue_end_ind] = this_indy;
                                queue_indx_last_negi[queue_end_ind] = iter->first;
                                queue_indy_last_negi[queue_end_ind] = iter->second;
                                queue_last_dis[queue_end_ind] = dist;
                                queue_size++;
                                queue_end_ind++;

                                label_mat(this_indx, this_indy) = label_count;
                                line_count_flag[this_indx] = true;

                                all_pushed_indx[all_pushed_ind_size] = this_indx;
                                all_pushed_indy[all_pushed_ind_size] = this_indy;
                                all_pushed_ind_size++;
                            }

                            else if ((iter->second == 0) && (queue_indy_last_negi[queue_start_ind] == 0)) // at the same beam
                            {
                                float dist_last = queue_last_dis[queue_start_ind];
                                if ((dist_last / dist <= 1.2) && ((dist_last / dist >= 0.8))) // inside a plane
                                {
                                    queue_indx[queue_end_ind] = this_indx;
                                    queue_indy[queue_end_ind] = this_indy;
                                    queue_indx_last_negi[queue_end_ind] = iter->first;
                                    queue_indy_last_negi[queue_end_ind] = iter->second;
                                    queue_last_dis[queue_end_ind] = dist;
                                    queue_size++;
                                    queue_end_ind++;

                                    label_mat(this_indx, this_indy) = label_count;
                                    line_count_flag[this_indx] = true;

                                    all_pushed_indx[all_pushed_ind_size] = this_indx;
                                    all_pushed_indy[all_pushed_ind_size] = this_indy;
                                    all_pushed_ind_size++;
                                }
                            }
                        }
                    }

                    bool feasible_segment = false;
                    if (all_pushed_ind_size >= min_cluster_size) // cluster_size > min_cluster_size
                    {
                        feasible_segment = true;
                    }
                    else if (all_pushed_ind_size >= segment_valid_point_num) // line_size > line_mini_size
                    {
                        int line_count = 0;
                        for (size_t i = 0; i < N_SCAN; i++)
                            if (line_count_flag[i])
                                line_count++;

                        if (line_count >= segment_valid_line_num)
                            feasible_segment = true;
                    }

                    if (feasible_segment)
                    {
                        label_count++;
                    }
                    else
                    {
                        for (size_t i = 0; i < all_pushed_ind_size; ++i)
                        {
                            label_mat(all_pushed_indx[i], all_pushed_indy[i]) = 999999;
                            range_mat(all_pushed_indx[i], all_pushed_indy[i]) = FLT_MAX;
                        }
                    }
                }
            }
        }
    }

    // remove noisy points
    void projectCloud(const pcl::PointCloud<PointXYZIRT>::Ptr &laser_cloud_in,
                            pcl::PointCloud<PointType> &cloud_matrix,
                            Eigen::MatrixXf &range_mat,
                            std::vector<pcl::PointCloud<PointType>> &cloud_scan,
                            std::vector<int> &cloud_scan_order)
    {
        // convert point cloud to a range image
        float horizon_angle, range;
        int row_id, column_id;
        for (size_t i = 0; i < laser_cloud_in->size(); i++)
        {
            PointType point;
            point.x = laser_cloud_in->points[i].x;
            point.y = laser_cloud_in->points[i].y;
            point.z = laser_cloud_in->points[i].z;
            point.intensity = laser_cloud_in->points[i].intensity;

            range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            row_id = laser_cloud_in->points[i].ring;
            if (row_id < 0 || row_id >= N_SCAN * downsampleRate)
                continue;

            if (row_id % downsampleRate != 0)
                continue;

            row_id = row_id / downsampleRate;

            horizon_angle = atan2(point.x, point.y) * 180 / M_PI;
            static float ang_res_x = 360.0 / float(Horizon_SCAN);
            column_id = -round((horizon_angle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            
            if (column_id >= Horizon_SCAN)
                column_id -= Horizon_SCAN;
            if (column_id < 0 || column_id >= Horizon_SCAN)
                continue;
            if (range_mat(row_id, column_id) != FLT_MAX)
                continue;
            
            point = deskewPoint(&point, laser_cloud_in->points[i].time);

            point.intensity += row_id;
            int index = column_id + row_id * Horizon_SCAN;
            cloud_matrix.points[index] = point;
            range_mat(row_id, column_id) = range;

            cloud_scan[row_id].push_back(point); // without changing the point order
            cloud_scan_order[index] = cloud_scan[row_id].size() - 1;

            fullCloud->points[index] = point;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Lidar Cloud Deskew Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}