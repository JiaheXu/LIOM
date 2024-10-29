//
// Created by shibo zhao on 2020-09-27.
//

#include "super_odometry/ScanRegistration/scanRegistration.h"

namespace superodom
{

    scanRegistration::scanRegistration(const rclcpp::NodeOptions & options)
    : Node("scan_registration_node", options) {
    }
    
    void scanRegistration::initInterface() {
        if (!readParameters())
        {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::scanRegistration] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }
        if (!readCalibration(shared_from_this()))
        {
            RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::scanRegistration] Could not read parameters. Exiting...");
            rclcpp::shutdown();
        }

        RCLCPP_WARN_STREAM(this->get_logger(), "T_i_l: " << T_i_l);

        if (config_.N_SCANS != 16 && config_.N_SCANS != 32 && config_.N_SCANS != 64)
        {
            RCLCPP_ERROR(this->get_logger(), "only support velodyne with 16, 32 or 64 scan line!");
            rclcpp::shutdown();
        }

        subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            LASER_TOPIC, 2, 
            std::bind(&scanRegistration::laserCloudHandler, this,
                        std::placeholders::_1));

        subImu = this->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, 10, 
            std::bind(&scanRegistration::imu_Handler, this,
                        std::placeholders::_1));

        subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
            ODOM_TOPIC, 10, 
            std::bind(&scanRegistration::visual_odom_Handler, this,
                        std::placeholders::_1));
        
        pubLaserFeatureInfo = this->create_publisher<super_odometry_msgs::msg::LaserFeature>(
            ProjectName+"/feature_info", 2);
        
        pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_cloud_2", 2);

        pubCornerPointsSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_sharp", 2);

        pubCornerPointsLessSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_less_sharp", 2);

        pubSurfPointsFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_flat", 2);

        pubSurfPointsLessFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_less_flat", 2);

        pubRemovePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_remove_points", 2);

        if (PUB_EACH_LINE)
        {
            for (int i = 0; i < config_.N_SCANS; i++)
            {
                auto tmp_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    "laser_scanid_" + std::to_string(i), 2);
                pubEachScan.push_back(tmp_publisher_);
            }
        }
        delay_count_ = 0;
    }

    bool scanRegistration::readParameters()
    {
        this->declare_parameter<int>("scan_line");
        this->declare_parameter<int>("mapping_skip_frame");
        this->declare_parameter<double>("blindFront");
        this->declare_parameter<double>("blindBack");
        this->declare_parameter<double>("blindLeft");
        this->declare_parameter<double>("blindRight");
        this->declare_parameter<double>("blindUp");
        this->declare_parameter<double>("blindDown");
        this->declare_parameter<float>("min_range");
        this->declare_parameter<float>("max_range");
        this->declare_parameter<std::string>("imu_topic");
        this->declare_parameter<std::string>("odom_topic");
        this->declare_parameter<std::string>("laser_topic");
        this->declare_parameter<int>("downsample_rate");
        this->declare_parameter<int>("skip_point");

        if (!this->get_parameter("scan_line", config_.N_SCANS)) {
            return false;
        }
        if (!this->get_parameter("mapping_skip_frame", config_.skipFrame)) {
            return false;
        }
        if (!this->get_parameter("blindFront", config_.box_size.blindFront)) {
            return false;
        }
        if (!this->get_parameter("blindBack", config_.box_size.blindBack)) {
            return false;
        }
        if (!this->get_parameter("blindLeft", config_.box_size.blindLeft)) {
            return false;
        }
        if (!this->get_parameter("blindRight", config_.box_size.blindRight)) {
            return false;
        }
        if (!this->get_parameter("blindUp", config_.box_size.blindUp)) {
            return false;
        }
        if (!this->get_parameter("blindDown", config_.box_size.blindDown)) {
            return false;
        }
        if (!this->get_parameter("min_range", config_.min_range)) {
            return false;
        }
        if (!this->get_parameter("max_range", config_.max_range)) {
            return false;
        }
        if (!this->get_parameter("imu_topic", config_.IMU_TOPIC)) {
            return false;
        }
        if (!this->get_parameter("odom_topic", config_.ODOM_TOPIC)) {
            return false;
        }
        if (!this->get_parameter("laser_topic", config_.LASER_TOPIC)) {
            return false;
        }
        if (!this->get_parameter("downsample_rate", config_.downsampleRate)) {
            return false;
        }
        if (!this->get_parameter("skip_point", config_.skip_point)) {
            return false;
        }

        std::string sensorStr;
        if (!this->get_parameter("sensor", sensorStr))
        {
            RCLCPP_ERROR(this->get_logger(), "no sensor parameters");
            return false;
        }
        if (sensorStr == "velodyne")
        {
            config_.sensor = SensorType::VELODYNE;
            std::cerr << "sensor = VELODYNE" << std::endl;
        }
        else if (sensorStr == "ouster")
        {
            config_.sensor = SensorType::OUSTER;
            std::cerr << "sensor=OUSTER" << std::endl;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid sensor type: %s. (must be either 'velodyne' or 'ouster')", sensorStr.c_str());
            rclcpp::shutdown();
        }

        return true;
    }

    template <typename PointT>
    void scanRegistration::removeClosestFarestPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                                            pcl::PointCloud<PointT> &cloud_out, float min_range,
                                                            float max_range)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        // RCLCPP_WARN(this->get_logger(),"blindFront: %f", config_.box_size.blindFront);
        // RCLCPP_WARN(this->get_logger(),"blindBack: %f", config_.box_size.blindBack);
        // RCLCPP_WARN(this->get_logger(),"blindRight: %f", config_.box_size.blindRight);
        // RCLCPP_WARN(this->get_logger(),"blindLeft: %f", config_.box_size.blindLeft);

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            //In the bounding box filter
            if (cloud_in.points[i].x > config_.box_size.blindBack && cloud_in.points[i].x < config_.box_size.blindFront &&
                cloud_in.points[i].y > config_.box_size.blindRight && cloud_in.points[i].y < config_.box_size.blindLeft && 
                cloud_in.points[i].z < config_.box_size.blindUp && cloud_in.points[i].z > config_.box_size.blindDown)
            {
                continue;
            }

            if (cloud_in.points[i].x * cloud_in.points[i].x +
                    cloud_in.points[i].y * cloud_in.points[i].y +
                    cloud_in.points[i].z * cloud_in.points[i].z <
                min_range * min_range)
                continue;

            if (cloud_in.points[i].x * cloud_in.points[i].x +
                    cloud_in.points[i].y * cloud_in.points[i].y +
                    cloud_in.points[i].z * cloud_in.points[i].z >
                max_range * max_range)
                continue;

            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    template <typename Meas>
    bool scanRegistration::synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                                    MapRingBuffer<pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr> &lidarBuf,
                                                    bool remove_laserscan)
    {

        if (lidarBuf.empty() or measureBuf.empty())
            return false;

        //obtain the current laser message
        double lidar_start_time;
        lidarBuf.getFirstTime(lidar_start_time);

        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;

        lidarBuf.getFirstMeas(lidar_msg);

        double lidar_end_time = lidar_start_time + lidar_msg->back().time;

        // obtain the current imu message
        double meas_first_time;
        measureBuf.getFirstTime(meas_first_time);

        double meas_latest_time;
        measureBuf.getLastTime(meas_latest_time);

        // RCLCPP_INFO_STREAM(this->get_logger(),
        //     "check lidar front time: " << lidar_msg->front().time << "lidar back time: " << lidar_msg->back().time);

        // std::cout << std::fixed << "meas_first_time: " << meas_first_time << " meas_latest_time: " << meas_latest_time
        //           << " lidar_start_time:" << lidar_start_time << " lidar_end_time: " << lidar_end_time << std::endl;

        if (meas_first_time > lidar_start_time && remove_laserscan == true)
        {
            lidarBuf.clean(lidar_start_time);
            RCLCPP_WARN_STREAM(this->get_logger(),"meas_first_time > lidar_start_time");
            return false;
        }

        if (meas_first_time > lidar_start_time && remove_laserscan == false)
        {
            RCLCPP_WARN_STREAM(this->get_logger(),"meas_first_time > lidar_start_time || "
                            "message order is not perfect! please restart velodyne and imu driver!");
            return false;
        }

        if (meas_latest_time < lidar_end_time)
        {
            RCLCPP_WARN_STREAM(this->get_logger(),"meas_latest_time < lidar_end_time ||"
                            " message order is not perfect! please restart velodyne and imu driver!");
            return false;
        }

        // std::cout << std::fixed << "synchronization: "
        //           << "lidar_start_time： " << lidar_start_time
        //           << "lidar_msg.size(): " << lidar_msg->size() << std::endl;

        lidarBuf.getFirstTime(lidar_start_time);
        lidarBuf.getFirstMeas(lidar_msg);

        return true;
    }
    #pragma GCC push_options
        #pragma GCC optimize ("O0")
    void scanRegistration::imuRemovePointDistortion(double lidar_start_time, double lidar_end_time,
                                                    MapRingBuffer<Imu::Ptr> &imuBuf,
                                                    pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg)
    {

        TicToc t_whole;
        TicToc t_prepare;
        // RCLCPP_INFO(this->get_logger(),"imuRemovePointDistortion");
        auto &laserCloudIn = *lidar_msg;

        //  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

        double scan_dt = lidar_msg->back().time;

        Eigen::Quaterniond q_w_original;
        {
            double t_b_i = lidar_start_time;

            
            //    RCLCPP_INFO_STREAM(this->get_logger(),"a: " << std::to_string(t_b_i -
            //    before_ptr->second->time)); RCLCPP_INFO_STREAM(this->get_logger(),"b: " <<
            //    std::to_string(after_ptr->second->time - t_b_i)); RCLCPP_INFO_STREAM(this->get_logger(),"c:
            //    " << std::to_string(after_ptr->second->time -
            //                                            before_ptr->second->time));
            Eigen::Quaterniond q_w_original = imuBuf.measMap_.begin()->second->q_w_i;

            auto after_ptr = imuBuf.measMap_.upper_bound(t_b_i);
            if(after_ptr->first>1.0)
            {
                auto before_ptr = after_ptr;
                before_ptr--;
                if(before_ptr->first>1.0)
                {
                    auto ratio_bi = before_ptr == after_ptr ? 0.0 : (t_b_i - before_ptr->second->time) / (after_ptr->second->time - before_ptr->second->time);
                    Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;
                    Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                    q_w_original = q_w_i_before.slerp(ratio_bi, q_w_i_after);
                }
            }

        }

        q_w_original_l = q_w_original * T_i_l.rot;
        q_w_original_l.normalized();
        t_w_original_l.x() = 0.0;
        t_w_original_l.y() = 0.0;
        t_w_original_l.z() = 0.0;

        //RCLCPP_WARN(this->get_logger(),"q_w_original_l: %f %f %f %f", q_w_original_l.w(), q_w_original_l.x(), q_w_original_l.y(), q_w_original_l.z());
        //RCLCPP_WARN_STREAM(this->get_logger(),"T_i_l: " << T_i_l);


        Eigen::Quaterniond q_w_end;
        {
            double t_b_i = lidar_end_time;


            auto after_ptr = imuBuf.measMap_.upper_bound(t_b_i);
            if(after_ptr->first>1.0)
            {
                auto before_ptr = after_ptr;
                before_ptr--;
                if(before_ptr->first>1.0)
                {
                    auto ratio_bi = before_ptr == after_ptr ? 0.0 : (t_b_i - before_ptr->second->time) / (after_ptr->second->time - before_ptr->second->time);
                    Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;
                    Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                    q_w_end = q_w_i_before.slerp(ratio_bi, q_w_i_after);
                }
            }



            //    RCLCPP_INFO_STREAM(this->get_logger(),"end a: " << std::to_string(t_b_i -
            //    before_ptr->second->time)); RCLCPP_INFO_STREAM(this->get_logger(),"end b: " <<
            //    std::to_string(after_ptr->second->time - t_b_i)); RCLCPP_INFO_STREAM(this->get_logger(),"end
            //    c: " << std::to_string(after_ptr->second->time -
            //        before_ptr->second->time));
            
        }

        Eigen::Quaterniond q_w_end_l = q_w_original * T_i_l.rot;
        q_w_end_l.normalized();

        Eigen::Quaterniond q_original_end = q_w_original.inverse() * q_w_end;
        q_original_end.normalized();

        //RCLCPP_WARN(this->get_logger(),"transforming pointcloud");
        
        for (auto &point : laserCloudIn)
        {
            double t_b_i = point.time + lidar_start_time;
            Eigen::Quaterniond q_w_i = imuBuf.measMap_.begin()->second->q_w_i;

            auto after_ptr = imuBuf.measMap_.upper_bound(t_b_i);
            if(after_ptr->first>1.0)
            {
                auto before_ptr = after_ptr;
                before_ptr--;
                if(before_ptr->first>1.0)
                {
                    auto ratio_bi = before_ptr == after_ptr ? 0.0 : (t_b_i - before_ptr->second->time) / (after_ptr->second->time - before_ptr->second->time);
                    Eigen::Quaterniond q_w_i_after = after_ptr->second->q_w_i;
                    Eigen::Quaterniond q_w_i_before = before_ptr->second->q_w_i;
                    q_w_i = q_w_i_before.slerp(ratio_bi, q_w_i_after);
                }
            }
            

            Eigen::Quaterniond q_original_i = q_w_original.inverse() * q_w_i;

            Transformd T_original_i(q_original_i, Eigen::Vector3d::Zero());
            Transformd T_original_i_l = T_l_i * T_original_i * T_i_l;

            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                Eigen::Vector3d pt{point.x, point.y, point.z};
                pt = T_i_l* T_original_i_l * pt;

                point.x = pt.x();
                point.y = pt.y();
                point.z = pt.z();
            }
        }
    }
        #pragma GCC pop_options

    bool scanRegistration::checkExclusionZone(const PointType& p)
    {
        double MAX_Z = 0.4;
        double MIN_Z = -0.5;
        double MAX_ANGLE = M_PI/4;
        double MAX_X = 5; 
        
        double angle = atan2(p.y,p.x);

        if(p.z < MAX_Z && p.z > MIN_Z && p.x < MAX_X && std::abs(angle) < MAX_ANGLE)
            return true;

        if(p.z < MAX_Z && p.z > MIN_Z && p.x > -MAX_X && angle < M_PI + MAX_ANGLE && angle > M_PI - MAX_ANGLE)
            return true;

        return false;
    }

    void scanRegistration::feature_extraction(double lidar_start_time,
                                                pcl::PointCloud<point_os::PointcloudXYZITR> &laserCloudIn)
    {
        TicToc t_whole;
        TicToc t_prepare;

        double scan_dt = laserCloudIn.back().time;

        removeClosestFarestPointCloud(laserCloudIn, laserCloudIn, config_.min_range, config_.max_range);

        //   removeClosedPointCloud(laserCloudIn, laserCloudIn, config_.min_range);
        // //print("end Ori %f\n", endOri);
        size_t cloudSize = laserCloudIn.size();

        // //print("points size %ld \n", cloudSize);

        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(config_.N_SCANS);

        // //print("the number of config_.N_SCANS %d  \n", config_.N_SCANS);
        for (const auto &pt : laserCloudIn)
        {
        
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            
            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {     
            int scanID = pt.ring;

            if (scanID < 0 || scanID >= config_.N_SCANS)
                continue;

            if (scanID % config_.downsampleRate != 0)
                continue;

            float relTime = pt.time / scan_dt;
            point.intensity = scanID + scanPeriod * relTime;
            int newscanID=int(scanID/config_.downsampleRate);
        //     //print("new scanid %d \n", newscanID);
            laserCloudScans[newscanID].push_back(point);
            }
        }


        std::vector<int> scanStartInd(config_.N_SCANS, 0);
        std::vector<int> scanEndInd(config_.N_SCANS, 0);

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < config_.N_SCANS/config_.downsampleRate; i++)
        {
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        
        }
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*laserCloud, *laserCloud, indices);    


        for (size_t i = 5; i < cloudSize - 5; i++)
        {
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x +
                            laserCloud->points[i - 3].x + laserCloud->points[i - 2].x +
                            laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
                            laserCloud->points[i + 1].x + laserCloud->points[i + 2].x +
                            laserCloud->points[i + 3].x + laserCloud->points[i + 4].x +
                            laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y +
                            laserCloud->points[i - 3].y + laserCloud->points[i - 2].y +
                            laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
                            laserCloud->points[i + 1].y + laserCloud->points[i + 2].y +
                            laserCloud->points[i + 3].y + laserCloud->points[i + 4].y +
                            laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z +
                            laserCloud->points[i - 3].z + laserCloud->points[i - 2].z +
                            laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
                            laserCloud->points[i + 1].z + laserCloud->points[i + 2].z +
                            laserCloud->points[i + 3].z + laserCloud->points[i + 4].z +
                            laserCloud->points[i + 5].z;
            
            
            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            // //print("i: %d, cloud size: %d curvature: %f \n", i, cloudSize - 5,cloudCurvature[i]);
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        
        
        }
        

        TicToc t_pts;

        pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
        cornerPointsSharp->reserve(cloudSize);
        pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
        cornerPointsLessSharp->reserve(cloudSize);
        pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
        surfPointsFlat->reserve(cloudSize);
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());
        surfPointsLessFlat->reserve(cloudSize);

        float t_q_sort = 0;
        for (int i = 0; i < config_.N_SCANS/config_.downsampleRate; i++)
        {
            if (scanEndInd[i] - scanStartInd[i] < 6)
                continue;
            pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(
                new pcl::PointCloud<PointType>);
            for (int j = 0; j < 6; j++)
            {
                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
                int ep =
                    scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                TicToc t_tmp;
                std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);
                t_q_sort += t_tmp.toc();

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSortInd[k];

                    if(checkExclusionZone(laserCloud->points[ind])) continue;


                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 10.0)
                    {

                        largestPickedNum++;
                        if (largestPickedNum <= 2)
                        {
                            cloudLabel[ind] = 2;
                            cornerPointsSharp->push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20)
                        {
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                            // RCLCPP_INFO(this->get_logger(),"cloudCurvature: %f", cloudCurvature[ind]);
                        }
                        else
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x -
                                            laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y -
                                            laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z -
                                            laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x -
                                            laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y -
                                            laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z -
                                            laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSortInd[k];

                    if(checkExclusionZone(laserCloud->points[ind])) continue;


                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                    {

                        cloudLabel[ind] = -1;
                        surfPointsFlat->push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x -
                                            laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y -
                                            laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z -
                                            laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x -
                                            laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y -
                                            laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z -
                                            laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                    {
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                    }
                }
            }

            pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            *surfPointsLessFlat += surfPointsLessFlatScanDS;
        }

        //print("sort q time %f \n", t_q_sort);
        //print("seperate points time %f \n", t_pts.toc());

        {

            // point.x = q_w_original_l.x();
            // point.y = q_w_original_l.y();
            // point.z = q_w_original_l.z();
            // point.data[3] = q_w_original_l.w();
            // //    point.data[0]=t_w_original_l.x();
            // //   point.data[1]=t_w_original_l.y();
            // //   point.data[2]=t_w_original_l.z();
            // laserCloud->push_back(point);
        }

        // pcl::VoxelGrid<PointType> downSizeFilter;
        // downSizeFilter.setInputCloud(laserCloud);
        // downSizeFilter.setLeafSize(0.4, 0.4, 0.4);
        // downSizeFilter.filter(*laserCloud);

        // RCLCPP_INFO(this->get_logger(),"finished feature extraction");
    # if 1
        // T_original_i_l = T_i_l * T_original_i_l;
        q_w_original_l = q_w_original_l * T_i_l.rot.inverse();
        //RCLCPP_WARN(this->get_logger(),"q_w_original_l: %f %f %f %f", q_w_original_l.w(), q_w_original_l.x(), q_w_original_l.y(), q_w_original_l.z());
        q_w_original_l.normalize();
        //wrap feature message in one message
        FeatureHeader.frame_id = "sensor_init";
        FeatureHeader.stamp = rclcpp::Time(lidar_start_time*1e9);
        laserFeature.header = FeatureHeader;
        laserFeature.odom_available = false;
        laserFeature.cloud_nodistortion = publishCloud(pubLaserCloud, laserCloud, FeatureHeader.stamp, "sensor");
        laserFeature.cloud_corner = publishCloud(pubCornerPointsLessSharp, cornerPointsLessSharp, FeatureHeader.stamp, "sensor");
        laserFeature.cloud_surface = publishCloud(pubSurfPointsLessFlat, surfPointsLessFlat, FeatureHeader.stamp, "sensor");
        laserFeature.initial_quaternion_x = q_w_original_l.x();
        laserFeature.initial_quaternion_y = q_w_original_l.y();
        laserFeature.initial_quaternion_z = q_w_original_l.z();
        laserFeature.initial_quaternion_w= q_w_original_l.w();
        laserFeature.imu_available = true;
        laserFeature.sensor = 1; //Ouster
        pubLaserFeatureInfo->publish(laserFeature);

    #else

        sensor_msgs::msg::PointCloud2 laserCloudOutMsg;
        pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = rclcpp::Time(lidar_start_time*1e9);
        laserCloudOutMsg.header.frame_id = "sensor_init";
        pubLaserCloud->publish(laserCloudOutMsg);

        sensor_msgs::msg::PointCloud2 cornerPointsLessSharpMsg;
        pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharpMsg);
        cornerPointsLessSharpMsg.header.stamp = rclcpp::Time(lidar_start_time*1e9);
        cornerPointsLessSharpMsg.header.frame_id = "sensor_init";
        pubCornerPointsLessSharp->publish(cornerPointsLessSharpMsg);


        sensor_msgs::msg::PointCloud2 surfPointsLessFlat2;
        pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
        surfPointsLessFlat2.header.stamp = rclcpp::Time(lidar_start_time*1e9);
        surfPointsLessFlat2.header.frame_id = "sensor_init";
        pubSurfPointsLessFlat->publish(surfPointsLessFlat2);
    #endif


        sensor_msgs::msg::PointCloud2 cornerPointsSharpMsg;
        pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharpMsg);
        cornerPointsSharpMsg.header.stamp = rclcpp::Time(lidar_start_time*1e9);
        cornerPointsSharpMsg.header.frame_id = "sensor_init";
        pubCornerPointsSharp->publish(cornerPointsSharpMsg);

        sensor_msgs::msg::PointCloud2 surfPointsFlat2;
        pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
        surfPointsFlat2.header.stamp = rclcpp::Time(lidar_start_time*1e9);
        surfPointsFlat2.header.frame_id = "sensor_init";
        pubSurfPointsFlat->publish(surfPointsFlat2);

        // pub each scam
        if (PUB_EACH_LINE)
        {
            for (int i = 0; i < config_.N_SCANS; i++)
            {
                sensor_msgs::msg::PointCloud2 scanMsg;
                pcl::toROSMsg(laserCloudScans[i], scanMsg);
                scanMsg.header.stamp = rclcpp::Time(lidar_start_time*1e9);
                scanMsg.header.frame_id = "sensor_init";
                pubEachScan[i]->publish(scanMsg);
            }
        }

        //print("scan registration time %f ms *************\n", t_whole.toc());
        if (t_whole.toc() > 100)
            //RCLCPP_WARN(this->get_logger(),"scan registration process over 100ms");

        lidarBuf.clean(lidar_start_time);
    }

    sensor_msgs::msg::PointCloud2
    scanRegistration::publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud,
                                    rclcpp::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::msg::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        if (thisPub->get_subscription_count() != 0)
            thisPub->publish(tempCloud);
        return tempCloud;
    }

    sensor_msgs::msg::PointCloud2
    scanRegistration::publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<pcl::PointXYZHSV>::Ptr thisCloud,
                                    rclcpp::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::msg::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        if (thisPub->get_subscription_count() != 0)
            thisPub->publish(tempCloud);
        return tempCloud;
    }

    void scanRegistration::vioRemovePointDistortion(double lidar_start_time, double lidar_end_time,
                                                    MapRingBuffer<const nav_msgs::msg::Odometry::SharedPtr> &vioBuf,
                                                    pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr &lidar_msg)
    {

        auto &laserCloudIn = *lidar_msg;

        Eigen::Quaterniond q_w_original;

        double t_b_i = lidar_start_time;

        auto after_ptr = vioBuf.measMap_.upper_bound(t_b_i);
        auto before_ptr = after_ptr;
        before_ptr--;

        //    RCLCPP_INFO_STREAM(this->get_logger(),"a: " << std::to_string(t_b_i -
        //    before_ptr->second->time)); RCLCPP_INFO_STREAM(this->get_logger(),"b: " <<
        //    std::to_string(after_ptr->second->time - t_b_i)); RCLCPP_INFO_STREAM(this->get_logger(),"c:
        //    " << std::to_string(after_ptr->second->time -
        //                                            before_ptr->second->time));

        double ratio_bi = (t_b_i - before_ptr->first) /
                            (after_ptr->first - before_ptr->first);

        Eigen::Quaterniond q_w_i_before;
        q_w_i_before.x() = before_ptr->second->pose.pose.orientation.x;
        q_w_i_before.y() = before_ptr->second->pose.pose.orientation.y;
        q_w_i_before.z() = before_ptr->second->pose.pose.orientation.z;
        q_w_i_before.w() = before_ptr->second->pose.pose.orientation.w;

        Eigen::Quaterniond q_w_i_after;
        q_w_i_after.x() = after_ptr->second->pose.pose.orientation.x;
        q_w_i_after.y() = after_ptr->second->pose.pose.orientation.y;
        q_w_i_after.z() = after_ptr->second->pose.pose.orientation.z;
        q_w_i_after.w() = after_ptr->second->pose.pose.orientation.w;

        q_w_original =
            q_w_i_before.slerp(ratio_bi, q_w_i_after);

        q_w_original.normalized();

        Eigen::Vector3d t_w_i_before;
        t_w_i_before.x() = before_ptr->second->pose.pose.position.x;
        t_w_i_before.y() = before_ptr->second->pose.pose.position.y;
        t_w_i_before.z() = before_ptr->second->pose.pose.position.z;

        Eigen::Vector3d t_w_i_after;
        t_w_i_after.x() = after_ptr->second->pose.pose.position.x;
        t_w_i_after.y() = after_ptr->second->pose.pose.position.y;
        t_w_i_after.z() = after_ptr->second->pose.pose.position.z;

        Eigen::Vector3d t_w_orignal =
            (1 - ratio_bi) * t_w_i_before + ratio_bi * t_w_i_after;

        Transformd T_w_original_i(q_w_original, t_w_orignal);
    #if 1
        Transformd T_w_original_l = T_w_original_i * T_i_l;
        q_w_original_l = T_w_original_l.rot;
        t_w_original_l = T_w_original_l.pos;
        q_w_original_l.normalized();
    #else
        Transformd T_w_original_l = T_w_original_i;
        q_w_original_l = T_w_original_l.rot;

        q_w_original_l.normalized();

    #endif

        for (auto &point : laserCloudIn)
        {
            double t_b_i = point.time + lidar_start_time;

            auto after_ptr = vioBuf.measMap_.lower_bound(t_b_i);
            auto before_ptr = after_ptr--;

            double ratio_bi = (t_b_i - before_ptr->first) /
                                (after_ptr->first - before_ptr->first);

            Eigen::Quaterniond q_w_i_before;

            q_w_i_before.x() = before_ptr->second->pose.pose.orientation.x;
            q_w_i_before.y() = before_ptr->second->pose.pose.orientation.y;
            q_w_i_before.z() = before_ptr->second->pose.pose.orientation.z;
            q_w_i_before.w() = before_ptr->second->pose.pose.orientation.w;

            Eigen::Quaterniond q_w_i_after;
            q_w_i_after.x() = after_ptr->second->pose.pose.orientation.x;
            q_w_i_after.y() = after_ptr->second->pose.pose.orientation.y;
            q_w_i_after.z() = after_ptr->second->pose.pose.orientation.z;
            q_w_i_after.w() = after_ptr->second->pose.pose.orientation.w;

            Eigen::Quaterniond q_w_i;
            q_w_i =
                q_w_i_before.slerp(ratio_bi, q_w_i_after);

            q_w_i.normalized();

            Eigen::Vector3d t_w_i_before;
            t_w_i_before.x() = before_ptr->second->pose.pose.position.x;
            t_w_i_before.y() = before_ptr->second->pose.pose.position.y;
            t_w_i_before.z() = before_ptr->second->pose.pose.position.z;

            Eigen::Vector3d t_w_i_after;
            t_w_i_after.x() = after_ptr->second->pose.pose.position.x;
            t_w_i_after.y() = after_ptr->second->pose.pose.position.y;
            t_w_i_after.z() = after_ptr->second->pose.pose.position.z;

            Eigen::Vector3d t_w_i =
                (1 - ratio_bi) * t_w_i_before + ratio_bi * t_w_i_after;

            Transformd T_w_i(q_w_i, t_w_i);
            Transformd T_original_i = T_w_original_i.inverse() * T_w_i;
    #if 1
            Transformd T_original_i_l = T_l_i * T_original_i * T_i_l;
    #else
            Transformd T_original_i_l = T_original_i;
    #endif

            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                Eigen::Vector3d pt{point.x, point.y, point.z};
                pt = T_original_i_l * pt;

                point.x = pt.x();
                point.y = pt.y();
                point.z = pt.z();
            }
        }
    }

    void scanRegistration::undistortionAndscanregistration()
    {
        // RCLCPP_INFO(this->get_logger(),"undistortionAndscanregistration");
        LASER_IMU_SYNC_SCCUESS = synchronize_measurements<Imu::Ptr>(imuBuf, lidarBuf, true);

        // LASER_CAMERA_SYNC_SUCCESS=synchronize_measurements<nav_msgs::msg::Odometry::SharedPtr>(visualOdomBuf,lidarBuf,false);

        // RCLCPP_WARN(this->get_logger(),"LASER_IMU_SYNC_SCCUESS: %d", LASER_IMU_SYNC_SCCUESS);
        // RCLCPP_WARN(this->get_logger(),"imuBuf size: %d", imuBuf.measMap_.size());
        // RCLCPP_WARN(this->get_logger(),"lidarBuf size: %d", lidarBuf.measMap_.size());
        // if (LASER_IMU_SYNC_SCCUESS == true)
        if (LASER_IMU_SYNC_SCCUESS && imuBuf.measMap_.size() > 0 && lidarBuf.measMap_.size()>0)
        {
            double lidar_start_time;
            lidarBuf.getFirstTime(lidar_start_time);
            pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr lidar_msg;
            lidarBuf.getFirstMeas(lidar_msg);
            double lidar_end_time = lidar_start_time + lidar_msg->back().time;

            // if (LASER_IMU_SYNC_SCCUESS == true and LASER_CAMERA_SYNC_SUCCESS == true)
            // {
            //     RCLCPP_INFO(this->get_logger(),"\033[1;32m----> IMU ,VIO laserscan are synchronized!.\033[0m");

            //     vioRemovePointDistortion(lidar_start_time, lidar_end_time, visualOdomBuf, lidar_msg);
            // }

            // if (LASER_IMU_SYNC_SCCUESS == false and LASER_CAMERA_SYNC_SUCCESS == true)
            // {
            //     RCLCPP_INFO(this->get_logger(),"\033[1;32m----> VIO and laserscan are synchronized!.\033[0m");

            //     vioRemovePointDistortion(lidar_start_time, lidar_end_time, visualOdomBuf, lidar_msg);
            // }

            // if (LASER_IMU_SYNC_SCCUESS == true and LASER_CAMERA_SYNC_SUCCESS == false)
            // {
                // RCLCPP_INFO(this->get_logger(),"\033[1;32m----> IMU and laserscan are synchronized!.\033[0m");
                imuRemovePointDistortion(lidar_start_time, lidar_end_time, imuBuf, lidar_msg);
            // }

                //RCLCPP_WARN(this->get_logger(),"feature Extraction");
                feature_extraction(lidar_start_time, *lidar_msg);
            // }

            

            LASER_CAMERA_SYNC_SUCCESS = false;
            LASER_IMU_SYNC_SCCUESS = false;
        }
    }

    void scanRegistration::imu_Handler(const sensor_msgs::msg::Imu::SharedPtr msg_in)
    {
        auto msg = msg_in;

        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;

        Eigen::Vector3d accel;
        accel << msg->linear_acceleration.x, msg->linear_acceleration.y,
            msg->linear_acceleration.z;
        Eigen::Vector3d gyr;
        gyr << msg->angular_velocity.x, msg->angular_velocity.y,
            msg->angular_velocity.z;

        // RCLCPP_DEBUG(this->get_logger(), "get imu at time: %.6f", timestamp);
        Imu::Ptr imudata = std::make_shared<Imu>();

        imudata->time = timestamp;
        imudata->acc = accel;
        imudata->gyr = gyr;
        if (!imuBuf.empty())
        {
            const Eigen::Quaterniond rot_last = imuBuf.measMap_.rbegin()->second->q_w_i;
            Eigen::Vector3d gyr_last = imuBuf.measMap_.rbegin()->second->gyr;
            const double &time_last = imuBuf.measMap_.rbegin()->second->time;

            double dt = timestamp - time_last;
            //    RCLCPP_INFO_STREAM(this->get_logger(),"dt: " << dt);
            Eigen::Vector3d delta_angle = dt * 0.5 * (gyr + gyr_last);
            Eigen::Quaterniond delta_r =
                Sophus::SO3d::exp(delta_angle).unit_quaternion();

            Eigen::Quaterniond rot = rot_last * delta_r;
            rot.normalized();
            imudata->q_w_i = rot;
        }
        m_buf.lock();

        imuBuf.addMeas(imudata, timestamp);

        
        //  Remove oldest lidar data from the buffer
        double lidar_last_time = timestamp + 1.0;
        lidarBuf.getLastTime(lidar_last_time);

        double vio_last_time = 0;
        visualOdomBuf.getLastTime(vio_last_time);

        if (timestamp > lidar_last_time + 0.15 || vio_last_time > lidar_last_time + 0.11)
        {
            RCLCPP_INFO(this->get_logger(),"imu timestamp: %f, lidar ts: %f", timestamp, lidar_last_time);
            // RCLCPP_INFO(this->get_logger(),"vio timestamp: %f, lidar ts: %f", vio_last_time, lidar_last_time);
            RCLCPP_INFO(this->get_logger(),"llidar buf size: %d", lidarBuf.getSize());
            
            if(timestamp - lidar_last_time < 0.2){
                RCLCPP_INFO(this->get_logger(),"undistortion");
                undistortionAndscanregistration();
            }else{
                RCLCPP_WARN(this->get_logger(), "ouster lidar data is more than 0.2s old, will not process.");
            }
            
            while (lidarBuf.getSize() > 0)
            {
                // Remove oldest lidar data from the buffer
                double lidar_first_time;
                lidarBuf.getFirstTime(lidar_first_time);
                lidarBuf.clean(lidar_first_time);
            }
            // RCLCPP_INFO(this->get_logger(),"llidar buf size: %d", lidarBuf.getSize());
        }


        m_buf.unlock();
    }

    void scanRegistration::visual_odom_Handler(const nav_msgs::msg::Odometry::SharedPtr visualOdometry)
    {
        m_buf.lock();
        visualOdomBuf.addMeas(visualOdometry, visualOdometry->header.stamp.sec + visualOdometry->header.stamp.nanosec*1e-9);
        m_buf.unlock();
    }

    void scanRegistration::assignTimeforPointCloud(pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_)
    {

        size_t cloud_size = laserCloudIn_ptr_->size();

        pointCloudwithTime.reset(new pcl::PointCloud<point_os::PointcloudXYZITR>());
        pointCloudwithTime->reserve(cloud_size);

        point_os::PointcloudXYZITR point;
        for (size_t i = 0; i < cloud_size; i++)
        {
            point.x = laserCloudIn_ptr_->points[i].x;
            point.y = laserCloudIn_ptr_->points[i].y;
            point.z = laserCloudIn_ptr_->points[i].z;
            point.intensity = laserCloudIn_ptr_->points[i].intensity;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (config_.N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (config_.N_SCANS - 1) || scanID < 0)
                {
                    cloud_size--;
                    continue;
                }
            }

            point.ring = scanID;
            float rel_time = (columnTime * int(i / config_.N_SCANS) + laserTime * (i % config_.N_SCANS)) / scanPeriod;
            float pointTime = rel_time * scanPeriod;
            point.time = pointTime;
            pointCloudwithTime->push_back(point);
        }
    }

    void scanRegistration::laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        if (delay_count_++ <= 40)
        {
            return; 
        }


        frameCount = (frameCount + 1) % config_.skipFrame;
        if (frameCount != 0)
            return;

        pcl::PointCloud<point_os::PointcloudXYZITR>::Ptr pointCloud(
            new pcl::PointCloud<point_os::PointcloudXYZITR>());

        tmpOusterCloudIn.reset(new pcl::PointCloud<point_os::OusterPointXYZIRT>());

        // int skip_ring = 4;
        // if (PROVIDE_POINT_TIME)
        if (true)
        {

            if (config_.sensor == SensorType::VELODYNE)
            {
                RCLCPP_INFO(this->get_logger(),"velodyne");
                pcl::fromROSMsg(*laserCloudMsg, *pointCloud);

            }
            else if (config_.sensor == SensorType::OUSTER)
            {
                RCLCPP_INFO(this->get_logger(),"ouster");

                // Convert to Velodyne format
                pcl::fromROSMsg(*laserCloudMsg, *tmpOusterCloudIn);
                // pointCloud->points.resize(tmpOusterCloudIn->size());
                pointCloud->points.clear();
                pointCloud->is_dense = tmpOusterCloudIn->is_dense;
                auto last_time = tmpOusterCloudIn->points[0].t * 1.e-9f;
                for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
                {
                    if (i % config_.skip_point != 0)
                        continue;
                
                    auto &src = tmpOusterCloudIn->points[i];
                    if (src.ring % config_.downsampleRate != 0)
                        continue;
                    point_os::PointcloudXYZITR dst;
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = src.t * 1e-9f;
                    pointCloud->points.push_back(dst);
                    // if(dst.time < 0.00001)
                    //     continue;
                    // // if(dst.time - last_time < 0){
                    //     RCLCPP_WARN(this->get_logger(),"--------------- TIMESTAMP WENT BACK");
                    //     RCLCPP_WARN(this->get_logger(),"dst time: %f", dst.time);
                    //     RCLCPP_WARN(this->get_logger(),"last time: %f", last_time);
                    // }
                    last_time = dst.time;
                }
                RCLCPP_INFO(this->get_logger(),"pointcloud size: %ld", pointCloud->points.size());
            }
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown sensor type: " << int(sensor));
                rclcpp::shutdown();
            }

        }
        else
        {
            pcl::PointCloud<PointType>::Ptr laserCloudIn_ptr_(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn_ptr_);
            assignTimeforPointCloud(laserCloudIn_ptr_);
            pointCloud = pointCloudwithTime;
        }

        m_buf.lock();

        if (lidarBuf.getSize() > 1)
        {
            RCLCPP_WARN(this->get_logger(),"lidar buf size: %d", lidarBuf.getSize());
            RCLCPP_WARN(this->get_logger(),"lidar not being processed.");
            double lidar_last_time = 0;
            lidarBuf.getLastTime(lidar_last_time);

            double vio_last_time = 0;
            visualOdomBuf.getLastTime(vio_last_time);

            double imu_last_time = 0;
            imuBuf.getLastTime(imu_last_time);

            RCLCPP_WARN(this->get_logger(),"lidar ts: %f, vo ts: %f, imu ts: %f", lidar_last_time, vio_last_time, imu_last_time);

            
        }

            if(lidarBuf.getSize()>10){
                
                RCLCPP_WARN(this->get_logger(),"lidarBuf too big, flushing everything out");
                while (lidarBuf.getSize() > 0)
                {
                    double lidar_first_time;
                    lidarBuf.getFirstTime(lidar_first_time);
                    lidarBuf.clean(lidar_first_time);
                }
            }
            

            lidarBuf.addMeas(pointCloud, laserCloudMsg->header.stamp.sec + laserCloudMsg->header.stamp.nanosec*1e-9);
            // RCLCPP_WARN(this->get_logger(),"lidar buf size: %d", lidarBuf.getSize());

            // undistortionAndscanregistration();

            m_buf.unlock();
        }

} // namespace superodom
