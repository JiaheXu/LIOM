// //
// // Created by shibo zhao on 2020-09-27.
// //

// #define AUTO_JACOBIAN 0
// #define DEBUG_VIEW 0
// #define ENABLE_GRAVITY 0
// #include "super_odometry/LaserMapping/laserMapping.h"

// double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
// Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
// Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);
// Eigen::Affine3d T_r_nr_ = Eigen::Affine3d(Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5));

// namespace superodom
// {

//     laserMapping::laserMapping() : Node("laser_mapping_node") {

//         if (!readParameters(this))
//         {
//             RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::laserMapping] Could not read parameters. Exiting...");
//             rclcpp::shutdown();
//         }

//         if (!readCalibration(this))
//         {
//             RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::laserMapping] Could not read parameters. Exiting...");
//             rclcpp::shutdown();
//         }

//         RCLCPP_INFO(this->get_logger(), "DEBUG VIEW: %d", config_.debug_view_enabled);
//         RCLCPP_INFO(this->get_logger(), "DEBUG VIEW: %d", config_.enable_ouster_data);
//         RCLCPP_INFO(this->get_logger(), "line resolution %f plane resolution %f vision_laser_time_offset %f",
//                 config_.lineRes, config_.planeRes, vision_laser_time_offset);


//         downSizeFilterCorner.setLeafSize(config_.lineRes, config_.lineRes, config_.lineRes);
//         downSizeFilterSurf.setLeafSize(config_.planeRes, config_.planeRes, config_.planeRes);

//         subLaserFeatureInfo = this->create_subscription<super_odometry_msgs::msg::LaserFeature>(
//             ProjectName+"feature_info", 2,
//             std::bind(&laserMapping::laserFeatureInfoCB,
//                         std::placeholders::_1));
//         subIMUOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
//             ProjectName+"integrated_to_init", 100,
//             std::bind(&laserMapping::imuOdometryCB
//                         std::placeholders::_1));
                        
//         // subVisualOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
//         //     "vins_estimator/imu_propagate", 100,
//         //     std::bind(&laserMapping::visualOdometryCB,
//         //                 std::placeholders::_1));

//         pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             ProjectName+"laser_cloud_surround", 2);
//         pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             ProjectName+"laser_cloud_map", 2);
//         pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             ProjectName+"velodyne_cloud_registered_imu", 2);
//         pubLaserCloudFullRes_rot = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             ProjectName+"velodyne_cloud_registered", 2);
//         pubLaserCloudFullResOusterWithFeatures = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             "ouster_cloud_registered_with_features", 1);
//         pubLaserCloudFullResOuster = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//             "ouster_cloud_registered", 1);
//         // TODO: Remember to add the ouster features

//         pubOdomAftMapped_rot = this->create_publisher<nav_msgs::msg::Odometry>(
//             ProjectName+"aft_mapped_to_init", 1);
//         pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>(
//             ProjectName+"aft_mapped_to_init_imu", 1);
//         pubLaserOdometryIncremental = this->create_publisher<nav_msgs::msg::Odometry>(
//             ProjectName+"aft_mapped_to_init_incremental", 1);
//         pubOdomAftMappedHighFrec = this->create_publisher<nav_msgs::msg::Odometry>(
//             ProjectName+"aft_mapped_to_init_high_frec", 1);
//         pubLaserAfterMappedPath = this->create_publisher<nav_msgs::msg::Path>(
//             ProjectName+"aft_mapped_path", 1);
//         pubOptimizationStats = this->create_publisher<super_odometry_msgs::msg::OptimizationStats>(
//             ProjectName+"super_odometry_stats", 1);

//         // pubprediction_source = this->create_publisher<std_msgs::msg::String>(
//         //     ProjectName+"prediction_source", 1);

//         if (config_.debug_view_enabled)
//         {
//             debug_view = new superodom::DebugView(pub_node, private_node);
//         }

//         localmap.lineRes_ = config_.lineRes;
//         localmap.planeRes_ = config_.planeRes;
//         localmap.forget_far_chunks_ = config_.forget_far_chunks;

//         slam.initROSInterface(shared_from_this());
//         slam.Visual_confidence_factor=config_.visual_confidence_factor;
//         slam.Pos_degeneracy_threshold=config_.pos_degeneracy_threshold;
//         slam.Ori_degeneracy_threshold=config_.ori_degeneracy_threshold;
//         slam.LocalizationICPMaxIter=config_.max_iterations;
//         slam.MaximumSurfFeatures=config_.max_surface_features;
        
//         timeLatestImuOdometry = rclcpp::Time(0);
//         timeLastMappingResult = rclcpp::Time(0);

//         initializationParam();
//         RCLCPP_WARN(this->get_logger(), "end constructor");

//     }


//     /* void laserMapping::setUpROS(ros::NodeHandle &pub_node, ros::NodeHandle &private_node)
//     // {

//     //     if (!readParameters(private_node))
//     //     {
//     //         RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::laserMapping] Could not read parameters. Exiting...");
//     //         ros::requestShutdown();
//     //     }

//     //     if(!readCalibration(private_node))
//     //     {
//     //         RCLCPP_ERROR(this->get_logger(), "[SuperOdometry::laserMapping] Could not read calibration. Exiting...");
//     //         ros::requestShutdown();
//     //     }else{
//     //         RCLCPP_WARN(this->get_logger(), "success reading global params");
//     //     }

//     //     RCLCPP_INFO(this->get_logger(), "DEBUG VIEW: %d", config_.debug_view_enabled);
//     //     RCLCPP_INFO(this->get_logger(), "DEBUG VIEW: %d", config_.enable_ouster_data);

//     //     RCLCPP_INFO(this->get_logger(), "line resolution %f plane resolution %f vision_laser_time_offset %f", config_.lineRes, config_.planeRes, vision_laser_time_offset);
//     //     downSizeFilterCorner.setLeafSize(config_.lineRes, config_.lineRes, config_.lineRes);
//     //     downSizeFilterSurf.setLeafSize(config_.planeRes, config_.planeRes, config_.planeRes);

//     //     subLaserFeatureInfo = this->create_subscription<super_odometry_msgs::msg::LaserFeature>("/superodom/feature_info", 10,
//     //                                                                                 &laserMapping::laserFeatureInfoCB,
//     //                                                                                 this,
//     //                                                                                 ros::TransportHints().tcpNoDelay());

//     //     subIMUOdometry = this->create_subscription<nav_msgs::msg::Odometry>("/integrated_to_init", 10,
//     //                                                             &laserMapping::imuOdometryCB, this);

//     //     // subLaserRawdata = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 2,
//     //     //                                                                &laserMapping::laserCloudRawDataCB,
//     //     //                                                                this);

//     //     // subTakeoffAlignment = this->create_subscription<takeoff_manager::TakeoffAlignment>("takeoff_alignment", 1, &laserMapping::takeoffAlignmentHandler, this);

//     //     pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_surround", 1);
//     //     pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_map", 1);
//     //     // pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_cloud_registered_imu", 1);
//     //     pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_cloud_registered_with_features", 1);
//     //     pubLaserCloudFullRes_rot = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_cloud_registered", 1);
//     //     pubLaserCloudFullResOusterWithFeatures = this->create_publisher<sensor_msgs::msg::PointCloud2>("ouster_cloud_registered_with_features", 1);
//     //     pubLaserCloudFullResOuster = this->create_publisher<sensor_msgs::msg::PointCloud2>("ouster_cloud_registered", 1);
//     //     pubLaserCloudRawRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_rawcloud_registered", 1);

//     //     pubOdomAftMapped_rot = this->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 1);
//     //     pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init_imu", 1);
//     //     pubLaserOdometryIncremental = this->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init_incremental", 1);
//     //     pubOdomAftMappedHighFrec = this->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init_high_frec", 1);
//     //     pubLaserAfterMappedPath = this->create_publisher<nav_msgs::msg::Path>("aft_mapped_path", 1);
//     //     pubOptimizationStats = this->create_publisher<super_odometry_msgs::msg::OptimizationStats>("super_odometry_stats", 1);

//     //     // pubPreviousCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("previous_cloud", 1, true);
//     //     // pubPreviousPose = this->create_publisher<nav_msgs::msg::Odometry>("previous_pose", 1, true);

//     //     if (config_.debug_view_enabled)
//     //     {
//     //         debug_view = new superodom::DebugView(pub_node, private_node);
//     //     }

//     //     localmap.lineRes_ = config_.lineRes;
//     //     localmap.planeRes_ = config_.planeRes;
//     //     localmap.forget_far_chunks_ = config_.forget_far_chunks;
//     //     slam.Visual_confidence_factor=config_.visual_confidence_factor;
//     //     slam.Pos_degeneracy_threshold=config_.pos_degeneracy_threshold;
//     //     slam.Ori_degeneracy_threshold=config_.ori_degeneracy_threshold;
//     //     slam.LocalizationICPMaxIter=config_.max_iterations;
//     //     slam.MaximumSurfFeatures=config_.max_surface_features;
        
//     //     timeLatestImuOdometry = rclcpp::Time(0);
//     //     timeLastMappingResult = rclcpp::Time(0);

//     //     initializationParam();
//     //     RCLCPP_WARN(this->get_logger(), "end setupROS");
//     // } */

//     void laserMapping::initializationParam()
//     {

//         laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
//         laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
//         laserCloudSurround.reset(new pcl::PointCloud<PointType>());
//         laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
//         laserCloudFullRes_rot.reset(new pcl::PointCloud<PointType>());
//         laserCloudRawRes.reset(new pcl::PointCloud<PointType>());
//         laserCloudCornerStack.reset(new pcl::PointCloud<PointType>());
//         laserCloudSurfStack.reset(new pcl::PointCloud<PointType>());
//         laserCloudRealsense.reset(new pcl::PointCloud<PointType>());
//         laserCloudRawWithFeatures.reset(new pcl::PointCloud<pcl::PointXYZHSV>());
//         velodyneLaserCloudRawWithFeatures.reset(new pcl::PointCloud<pcl::PointXYZHSV>());
//         ousterLaserCloudRawWithFeatures.reset(new pcl::PointCloud<pcl::PointXYZHSV>());

//         Eigen::Quaterniond q_wmap_wodom_(1, 0, 0, 0);
//         Eigen::Vector3d t_wmap_wodom_(0, 0, 0);
//         Eigen::Quaterniond q_wodom_curr_(1, 0, 0, 0);
//         Eigen::Vector3d t_wodom_curr_(0, 0, 0);
//         Eigen::Quaterniond q_wodom_pre_(1, 0, 0, 0);
//         Eigen::Vector3d t_wodom_pre_(0, 0, 0);

//         q_wmap_wodom = q_wmap_wodom_;
//         t_wmap_wodom = t_wmap_wodom_;
//         q_wodom_curr = q_wodom_curr_;
//         t_wodom_curr = t_wodom_curr_;
//         q_wodom_pre = q_wodom_pre_;
//         t_wodom_pre = t_wodom_pre_;

//         q_w_imu_pre = q_wmap_wodom_;
//         t_w_imu_pre = t_wmap_wodom_;

//         laser_incremental_T.rot = q_wmap_wodom_;
//         laser_incremental_T.pos = t_wmap_wodom_;

//         imu_odom_buf.allocate(5000);
//     }

//     bool laserMapping::readParameters(const rclcpp::Node::SharedPtr node_)
//     {
//         if (!node_->get_paramater("mapping_line_resolution", config_.lineRes))
//             return false;
//         if (!node_->get_paramater("mapping_plane_resolution", config_.planeRes))
//             return false;
//         if (!node_->get_paramater("max_iterations", config_.max_iterations))
//             return false;
//         if (!node_->get_paramater("debug_view", config_.debug_view_enabled))
//             return false;
//         if (!node_->get_paramater("enable_ouster_data", config_.enable_ouster_data))
//             return false;
//         if (!node_->get_paramater("publish_only_feature_points", config_.publish_only_feature_points))
//             return false;
//         if (!node_->get_paramater("use_imu_roll_pitch", config_.use_imu_roll_pitch))
//             return false;
//         // if (!node_->get_paramater("start_from_previous_map", config_.start_from_previous_map))
//         //     return false;
//         if (!node_->get_paramater("max_surface_features", config_.max_surface_features))
//             return false;
//         if (!node_->get_paramater("velocity_failure_threshold", config_.velocity_failure_threshold))
//             return false;
//         if (!node_->get_paramater("auto_voxel_size", config_.auto_voxel_size))
//             return false;
//         if (!node_->get_paramater("forget_far_chunks", config_.forget_far_chunks))
//             return false;

//         if (!node_->get_paramater("visual_confidence_factor", config_.visual_confidence_factor))
//             return false;

//         if (!node_->get_paramater("pos_degeneracy_threshold", config_.pos_degeneracy_threshold))
//             return false;

//         if (!node_->get_paramater("ori_degeneracy_threshold", config_.ori_degeneracy_threshold))
//             return false;

//         if (!node_->get_paramater("world_frame", config_.WORLD_FRAME))
//             return false;

//         if (!node_->get_paramater("world_frame_rot", config_.WORLD_FRAME_ROT))
//             return false;

//         if (!node_->get_paramater("sensor_frame", config_.SENSOR_FRAME))
//             return false;

//         return true;
//     }

//     void
//     laserMapping::transformAssociateToMap(Transformd T_w_pre, Transformd T_wodom_curr, Transformd T_wodom_pre)
//     {
// #if 1
//         Transformd T_wodom_pre_curr = T_wodom_pre.inverse() * T_wodom_curr;

//         Transformd T_w_curr_predict = T_w_pre * T_wodom_pre_curr;
//         q_w_curr = T_w_curr_predict.rot;
//         t_w_curr = T_w_curr_predict.pos;
// #else
//         q_w_curr = q_wmap_wodom * q_wodom_curr;
//         t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
// #endif
//     }

//     void laserMapping::transformAssociateToMap()
//     {
//         q_w_curr = q_wmap_wodom * q_wodom_curr;
//         t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
//     }

//     void laserMapping::transformUpdate()
//     {
//         q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
//         t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;

//         // q_wmap_wodom = q_w_curr * q_w_laserodom_curr.inverse();
//         // t_wmap_wodom = t_w_curr - q_wmap_wodom * t_w_laserodom_curr;
//     }

//     void laserMapping::pointAssociateToMap(PointType const *const pi, PointType *const po)
//     {
//         Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
//         Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
//         po->x = point_w.x();
//         po->y = point_w.y();
//         po->z = point_w.z();
//         po->intensity = pi->intensity;
//         // po->intensity = 1.0;
//     }

//     void laserMapping::pointAssociateToMap(pcl::PointXYZHSV const *const pi, pcl::PointXYZHSV *const po)
//     {
//         Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
//         Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
//         po->x = point_w.x();
//         po->y = point_w.y();
//         po->z = point_w.z();
//         po->h = pi->h;
//         po->s = pi->s;
//         po->v = pi->v;
//         // po->intensity = 1.0;
//     }

//     void laserMapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
//     {
//         Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
//         Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
//         po->x = point_curr.x();
//         po->y = point_curr.y();
//         po->z = point_curr.z();
//         po->intensity = pi->intensity;
//     }


//     void laserMapping::laserFeatureInfoCB(const super_odometry_msgs::msg::LaserFeatureConstPtr &msgIn)
//     {
//         // RCLCPP_WARN(this->get_logger(), "sensor type: %d, timestamp: %f ", (int) msgIn->sensor, msgIn->header.stamp.seconds());

//         // RCLCPP_WARN(this->get_logger(), "last sensor type: %d ", (int) last_sensor_type_);
//         if (last_sensor_type_ == SensorType::OUSTER && (int)msgIn->sensor == 1)
//             return;
//         // avoid repeated ouster data

//         mBuf.lock();
//         cornerLastBuf.push(msgIn->cloud_corner);
//         surfLastBuf.push(msgIn->cloud_surface);
//         realsenseBuf.push(msgIn->cloud_realsense);
//         // fullResBuf.push(msgIn->cloud_nodistortion);
//         rawWithFeaturesBuf.push(msgIn->cloud_nodistortion);

//         Eigen::Quaterniond imuprediction_tmp(msgIn->initial_quaternion_w, msgIn->initial_quaternion_x,
//                                              msgIn->initial_quaternion_y, msgIn->initial_quaternion_z);
//         IMUPredictionBuf.push(imuprediction_tmp);
//         sensorTypeLastBuf.push((SensorType)msgIn->sensor);
//         last_sensor_type_ = (SensorType) msgIn->sensor;

//         // RCLCPP_WARN(this->get_logger(), "sensor type: %d, timestamp: %f ", (int) msgIn->sensor, msgIn->header.stamp.seconds());
//         mBuf.unlock();
//     }

//     void laserMapping::laserCloudRawDataCB(const sensor_msgs::msg::PointCloud2::SharedPtr &laserCloudRawdata)
//     {

//         mBuf.lock();
//         if (rawDataBuf.size() < 5)
//         {
//             rawDataBuf.push(laserCloudRawdata);
//         }
//         mBuf.unlock();

//         // // RCLCPP_INFO(this->get_logger(), "waiting_takeoff_timeout: %d", waiting_takeoff_timeout);
//         // if (waiting_takeoff_timeout > 0)
//         // {
//         //     waiting_takeoff_timeout--;
//         // }
//         // else
//         // {
//         //     config_.start_from_previous_map = false;
//         // }
//     }

//     void laserMapping::imuOdometryCB(const nav_msgs::msg::Odometry::SharedPtr &imuOdometry)
//     {
//         mBuf.lock();
//         imu_odom_buf.addMeas(imuOdometry, imuOdometry->header.stamp.seconds());
//         timeLatestImuOdometry = imuOdometry->header.stamp;
//         mBuf.unlock();
//     }

//     void laserMapping::extractIMUOdometry(double timeLaserFrame, Transformd &T_w_lidar)
//     {

//         double t_b_i = timeLaserFrame;

//         auto after_ptr = imu_odom_buf.measMap_.upper_bound(t_b_i);
//         auto before_ptr = after_ptr;
//         before_ptr--;

//         double ratio_bi =
//             (t_b_i - before_ptr->first) / (after_ptr->first - before_ptr->first);

// #if DEBUG_VIEW
//         LOG(INFO) << "[frontend_msg->Tw_imu_meas] delta t"
//                   << (t_b_i - before_ptr->first);
//         LOG(INFO) << "[frontend_msg->Tw_imu_meas] delta t"
//                   << (after_ptr->first - t_b_i);
// #endif

//         Eigen::Quaterniond q_w_i_before;
//         q_w_i_before.x() = before_ptr->second->pose.pose.orientation.x;
//         q_w_i_before.y() = before_ptr->second->pose.pose.orientation.y;
//         q_w_i_before.z() = before_ptr->second->pose.pose.orientation.z;
//         q_w_i_before.w() = before_ptr->second->pose.pose.orientation.w;

//         Eigen::Quaterniond q_w_i_after;

//         q_w_i_after.x() = after_ptr->second->pose.pose.orientation.x;
//         q_w_i_after.y() = after_ptr->second->pose.pose.orientation.y;
//         q_w_i_after.z() = after_ptr->second->pose.pose.orientation.z;
//         q_w_i_after.w() = after_ptr->second->pose.pose.orientation.w;

//         Eigen::Quaterniond q_w_imu_cur = q_w_i_before.slerp(ratio_bi, q_w_i_after);

//         Eigen::Vector3d t_w_i_before;
//         t_w_i_before.x() = before_ptr->second->pose.pose.position.x;
//         t_w_i_before.y() = before_ptr->second->pose.pose.position.y;
//         t_w_i_before.z() = before_ptr->second->pose.pose.position.z;

//         Eigen::Vector3d t_w_i_after;
//         t_w_i_after.x() = after_ptr->second->pose.pose.position.x;
//         t_w_i_after.y() = after_ptr->second->pose.pose.position.y;
//         t_w_i_after.z() = after_ptr->second->pose.pose.position.z;

//         Eigen::Vector3d t_w_imu_cur = (1 - ratio_bi) * t_w_i_before + ratio_bi * t_w_i_after;

//         if (lastimuodomAvaliable == false)
//         {
//             q_w_imu_pre = q_w_imu_cur;
//             t_w_imu_pre = t_w_imu_cur;
//             lastimuodomAvaliable = true;
//         }
//         else
//         {
//             Transformd T_w_imu_cur(q_w_imu_cur, t_w_imu_cur);
//             Transformd T_w_imu_pre(q_w_imu_pre, t_w_imu_pre);

//             Transformd T_w_imu_pre_cur = T_w_imu_pre.inverse() * T_w_imu_cur;
//             T_w_lidar = T_w_lidar * T_w_imu_pre_cur;

//             if (use_imu_roll_pitch_this_step)
//             {
//                 T_w_lidar.rot = q_w_imu_cur;
//             }

//             q_w_imu_pre = q_w_imu_cur;
//             t_w_imu_pre = t_w_imu_cur;

//             Transformd curr_imu_T;
//             curr_imu_T.rot = q_w_imu_cur;
//             curr_imu_T.pos = t_w_imu_cur;
//             last_imu_T = curr_imu_T;
//         }

// #if DEBUG_VIEW
//         RCLCPP_INFO_STREAM(this->get_logger(), "last_T_w_lidar: " << last_T_w_lidar);
//         RCLCPP_INFO_STREAM(this->get_logger(), "increment IMU" << last_imu_T.inverse() * curr_imu_T);
//         RCLCPP_INFO_STREAM(this->get_logger(), "Proposed First Guess: " << last_imu_T.inverse() * curr_imu_T * last_T_w_lidar);
//         debug_view->addOptimizationIterationPose(last_imu_T.inverse() * curr_imu_T * last_T_w_lidar, timeLaserFrame);

//         // T_w_lidar = (last_imu_T.inverse() * curr_imu_T) * last_T_w_lidar;
//         // T_w_lidar = last_T_w_lidar;

// #endif
//     }

//     void laserMapping::selectposePrediction()
//     {

//         if(! imu_odom_buf.empty())
//         {

//             imuodom_available = false;

//             double imu_odom_first_time, imu_odom_last_time;
//             imu_odom_buf.getFirstTime(imu_odom_first_time);
//             imu_odom_buf.getLastTime(imu_odom_last_time);

//             if (imu_odom_last_time < timeLaserOdometry)
//             {

//                 imuodom_available = false;
//             }

//             if (imu_odom_first_time > timeLaserOdometry)
//             {
//                 imuodom_available = false;
//             }

//             if (imu_odom_first_time < timeLaserOdometry and imu_odom_last_time > timeLaserOdometry)
//                 imuodom_available = true;
//         }

//         // use the incremental imu odometry as the initial guess
//         if (imuodom_available == true)
//         {
//             // RCLCPP_WARN(this->get_logger(), "imuodom_available");
//             T_w_lidar.rot = q_w_curr;
//             T_w_lidar.pos = t_w_curr;
//             extractIMUOdometry(timeLaserOdometry, T_w_lidar);
//             return;
//         }

//         // use the incremental imu orientation as the initial guess
//         if (imuorientationAvailable == true)
//         {
//             // RCLCPP_WARN(this->get_logger(), "useimuorientation Available");
//             Eigen::Quaterniond q_map_curr_estimate = q_w_curr * q_wodom_pre.inverse() * q_wodom_curr;
//             q_map_curr_estimate.normalize();

//             T_w_lidar.rot = q_map_curr_estimate;
//             q_wodom_pre = q_wodom_curr;
//             return;
//         }

//         RCLCPP_WARN(this->get_logger(), "use previous pose as start guess");
//         //use the incremental laser odometry as the initial guess
//         // ( Actually, not function since we assume imu data must be obtained)
//         // T_w_lidar = last_T_w_lidar  * total_incremental_T;
//         T_w_lidar = last_T_w_lidar;
//     }

//     void laserMapping::setInitialGuess()
//     {

//         use_imu_roll_pitch_this_step = false;

//         if (force_initial_guess)      // set the first poses by hand if you know it.
//         {
//             // RCLCPP_WARN(this->get_logger(), " force initial guess");
//             T_w_lidar = forcedInitialGuess;
//             q_w_curr = T_w_lidar.rot;
//             t_w_curr = T_w_lidar.pos;
//             force_initial_guess = false;
//             initialization = true;
//             // RCLCPP_INFO_STREAM(this->get_logger(), "Start TF: " << T_w_lidar);
//             last_T_w_lidar = T_w_lidar;
//         }
//         else if (initialization == false) //directly hardset the imu rotation as the first pose
//         {
//             // RCLCPP_WARN(this->get_logger(), "initialize");
//             use_imu_roll_pitch_this_step = true;
//             if (use_imu_roll_pitch_this_step)
//             {
//                 // RCLCPP_WARN(this->get_logger(), "use_imu_roll_pitch");
//                 double roll, pitch, yaw;
//                 tf2::Quaternion orientation_curr(q_wodom_curr.x(), q_wodom_curr.y(), q_wodom_curr.z(), q_wodom_curr.w());
//                 tf2::Matrix3x3(orientation_curr).getRPY(roll, pitch, yaw);
//                 // RCLCPP_INFO(this->get_logger(), "yaw: %f", yaw);
//                 tf2::Quaternion yaw_quat;
//                 yaw_quat.setRPY(0, 0, -yaw); //make sure the yaw anlge is zero at the beginning
//                 RCLCPP_INFO(this->get_logger(), "Start roll, pitch, yaw %f, %f, %f", roll, pitch, yaw);
//                 tf2::Quaternion first_orientation;
//                 first_orientation = yaw_quat*orientation_curr;
//                 first_orientation = first_orientation;
                
//                 // auto euler = q_wodom_curr.toRotationMatrix().eulerAngles(0, 1, 2);
//                 // auto rot = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
//                 //            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
//                 //            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
//                 q_w_curr = Eigen::Quaterniond(first_orientation.w(), first_orientation.x(), first_orientation.y(), first_orientation.z());
//                 LOG(INFO) << "imu laser R" << imu_laser_R;
//                 auto q_extrinsic=Eigen::Quaterniond(imu_laser_R);
//                 q_extrinsic.normalize();
//                 RCLCPP_WARN(this->get_logger(), "q_extrinsic: %f %f %f %f", q_extrinsic.w(),q_extrinsic.x(), q_extrinsic.y(), q_extrinsic.z());

//                 RCLCPP_WARN(this->get_logger(), "q_w_curr_pre: %f %f %f %f", q_w_curr.w(),q_w_curr.x(), q_w_curr.y(), q_w_curr.z() );
                
//                 q_w_curr = q_extrinsic.inverse()*q_w_curr;
//                 RCLCPP_WARN(this->get_logger(), "q_w_curr_pre: %f %f %f %f", q_w_curr.w(),q_w_curr.x(), q_w_curr.y(), q_w_curr.z() );

//                 // q_w_curr = q_wodom_curr;
//                 q_wodom_pre = q_w_curr;
//                 T_w_lidar.rot=q_w_curr;
//               //  initialization = true;
//             }
//             else
//             {
//                 RCLCPP_WARN(this->get_logger(), "start from zero");
//                 q_w_curr = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
//                 q_wodom_pre = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
//                 T_w_lidar.rot=q_w_curr;
//               //  initialization = true;
//             }
//         }
//         else if ( (localizationCount > 0 || startupCount > 0))
//         {
//             RCLCPP_WARN(this->get_logger(), "localization/startup");
//             use_imu_roll_pitch_this_step = true;
//             selectposePrediction();
//             if(use_imu_roll_pitch_this_step){
//                q_w_curr = T_w_lidar.rot;
//             }else{
//                q_w_curr = last_T_w_lidar.rot;
//             }
//             t_w_curr = last_T_w_lidar.pos;
//             T_w_lidar.pos = t_w_curr;
//             T_w_lidar.rot = q_w_curr;
//             startupCount--;
//         }
//         else
//         {
//             if(config_.use_imu_roll_pitch)
//                 use_imu_roll_pitch_this_step = true;
//             // RCLCPP_WARN(this->get_logger(), "selectPosePrediction");
//             selectposePrediction();
//             q_w_curr = T_w_lidar.rot;
//             t_w_curr = T_w_lidar.pos;
//         }
//         // RCLCPP_INFO(this->get_logger(), "use_imu_roll_pitch_this_step: %d", use_imu_roll_pitch_this_step);
//     }

//     void laserMapping::publishOdometry()
//     {
//         //       covert to loam axis
//         nav_msgs::msg::Odometry odomAftMapped_rot;
//         odomAftMapped_rot.header.frame_id = config_.WORLD_FRAME_ROT;
//         odomAftMapped_rot.child_frame_id = "aft_mapped";
//         odomAftMapped_rot.header.stamp = rclcpp::Time(timeLaserOdometry);

//         q_w_curr.normalized();
//         odomAftMapped_rot.pose.pose.orientation.x = q_w_curr.y();
//         odomAftMapped_rot.pose.pose.orientation.y = q_w_curr.z();
//         odomAftMapped_rot.pose.pose.orientation.z = q_w_curr.x();
//         odomAftMapped_rot.pose.pose.orientation.w = q_w_curr.w();

//         odomAftMapped_rot.pose.pose.position.x = t_w_curr.y();
//         odomAftMapped_rot.pose.pose.position.y = t_w_curr.z();
//         odomAftMapped_rot.pose.pose.position.z = t_w_curr.x();
//         pubOdomAftMapped_rot->publish(odomAftMapped_rot);

//         nav_msgs::msg::Odometry odomAftMapped;
//         odomAftMapped.header.frame_id = config_.WORLD_FRAME;
//         odomAftMapped.child_frame_id = "aft_mapped";
//         odomAftMapped.header.stamp = rclcpp::Time(timeLaserOdometry);

//         odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
//         odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
//         odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
//         odomAftMapped.pose.pose.orientation.w = q_w_curr.w();

//         odomAftMapped.pose.pose.position.x = t_w_curr.x();
//         odomAftMapped.pose.pose.position.y = t_w_curr.y();
//         odomAftMapped.pose.pose.position.z = t_w_curr.z();
//         rclcpp::Time pub_time = odomAftMapped.header.stamp;
//         pubOdomAftMapped->publish(odomAftMapped);

//         //publish incremental odometry
//         nav_msgs::msg::Odometry laserOdomIncremental;

//         if (initialization == false)
//         {
//             laserOdomIncremental.header.stamp = rclcpp::Time(timeLaserOdometry);
//             laserOdomIncremental.header.frame_id = config_.WORLD_FRAME;
//             laserOdomIncremental.child_frame_id = "aft_mapped";
//             laserOdomIncremental.pose.pose.position.x = t_w_curr.x();
//             laserOdomIncremental.pose.pose.position.y = t_w_curr.y();
//             laserOdomIncremental.pose.pose.position.z = t_w_curr.z();
//             laserOdomIncremental.pose.pose.orientation.x = q_w_curr.x();
//             laserOdomIncremental.pose.pose.orientation.y = q_w_curr.y();
//             laserOdomIncremental.pose.pose.orientation.z = q_w_curr.z();
//             laserOdomIncremental.pose.pose.orientation.w = q_w_curr.w();
//         }
//         else
//         {

//             laser_incremental_T = T_w_lidar;
//             laser_incremental_T.rot.normalized();

//             laserOdomIncremental.header.stamp = rclcpp::Time(timeLaserOdometry);
//             laserOdomIncremental.header.frame_id = config_.WORLD_FRAME;
//             laserOdomIncremental.child_frame_id = "aft_mapped";
//             laserOdomIncremental.pose.pose.position.x = laser_incremental_T.pos.x();
//             laserOdomIncremental.pose.pose.position.y = laser_incremental_T.pos.y();
//             laserOdomIncremental.pose.pose.position.z = laser_incremental_T.pos.z();
//             laserOdomIncremental.pose.pose.orientation.x = laser_incremental_T.rot.x();
//             laserOdomIncremental.pose.pose.orientation.y = laser_incremental_T.rot.y();
//             laserOdomIncremental.pose.pose.orientation.z = laser_incremental_T.rot.z();
//             laserOdomIncremental.pose.pose.orientation.w = laser_incremental_T.rot.w();
//         }

//         pubLaserOdometryIncremental->publish(laserOdomIncremental);

//         geometry_msgs::msg::PoseStamped laserAfterMappedPose;
//         laserAfterMappedPose.header = odomAftMapped.header;
//         laserAfterMappedPose.pose = odomAftMapped.pose.pose;
//         laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
//         laserAfterMappedPath.header.frame_id = config_.WORLD_FRAME;
//         laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
//         pubLaserAfterMappedPath->publish(laserAfterMappedPath);

//         stats.header = odomAftMapped.header;

//         if (timeLatestImuOdometry.toSec() < 1.0)
//         {
//             timeLatestImuOdometry = pub_time;
//         }

//         ros::Duration latency = timeLatestImuOdometry - pub_time;
//         // RCLCPP_INFO_STREAM(this->get_logger(), "latency: " << latency);
//         stats.latency = latency.toSec() * 1000;
//         // RCLCPP_INFO_STREAM(this->get_logger(), "msg latency: " << stats.latency);
//         stats.n_iterations = stats.iterations.size();

//         // Avoid breaking rqt_multiplot
//         while (stats.iterations.size() < 4)
//         {
//             stats.iterations.push_back(super_odometry_msgs::msg::IterationStats());
//         }

//         pubOptimizationStats->publish(stats);
//         stats.iterations.clear();
//         timeLastMappingResult = rclcpp::Time(timeLaserOdometry);

        

//         static tf2_ros::TransformBroadcaster br(this);
//         // tf2::Transform transform;
//         geometry_msgs::msg::TransformStamped transform;
//         transform.header.stamp  = odomAftMapped.header.stamp;
//         transform.header.frame_id = config_.WORLD_FRAME;
//         // transform.child_frame_id = "aft_mapped";
//         tf2::Quaternion q;
//         transform.setOrigin(tf2::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));
//         q.setW(q_w_curr.w());
//         q.setX(q_w_curr.x());
//         q.setY(q_w_curr.y());
//         q.setZ(q_w_curr.z());
//         transform.setRotation(q);
//         br.sendTransform(transform);
//     }


//     void laserMapping::preprocessDualLidarFeatures(Transformd current_pose, SensorType sensor_type)
//      {
//             //support cannary drone ouster and velodyne setup

//              // std::cout << "dual scan buffer size: " << scanTransformBuf.size() << std::endl;
//              size_t SCAN_BUF_SIZE = 2;
//              if(cornerDualScanBuf.size() == SCAN_BUF_SIZE-1 && surfDualScanBuf.size() == SCAN_BUF_SIZE-1 &&
//                 fullResDualScanBuf.size() == SCAN_BUF_SIZE-1 && scanTransformBuf.size() == SCAN_BUF_SIZE-1) {

//                  // RCLCPP_WARN(this->get_logger(), "converting second scan");
//                  // request base scan, which is the current scan
//                  // auto baseCornerCloud = cornerDualScanBuf.front();
//                  // cornerDualScanBuf.pop();
//                  // auto baseSurfCloud = surfDualScanBuf.front();
//                  // surfDualScanBuf.pop();
//                  // auto baseFullResCloud = fullResDualScanBuf.front();
//                  // fullResDualScanBuf.pop();
//                  auto baseScanTransform = current_pose;
//                  // scanTransformBuf.pop();

//                  // merge to latest frame
//                  while(!cornerDualScanBuf.empty()) {
//                      auto prevCornerCloud = cornerDualScanBuf.front();
//                      cornerDualScanBuf.pop();
//                      auto prevSurfCloud = surfDualScanBuf.front();
//                      surfDualScanBuf.pop();
//                      auto prevFullResCloud = fullResDualScanBuf.front();
//                      fullResDualScanBuf.pop();
//                      auto prevScanTransform = scanTransformBuf.front();
//                      scanTransformBuf.pop();

//                      auto prevSensorType = sensorTypeBuf.front();
//                      sensorTypeBuf.pop();

//                      auto deltaTransform = baseScanTransform.inverse() * prevScanTransform;

//                      if (sensor_type == SensorType::VELODYNE)
//                      {
//                          // RCLCPP_WARN(this->get_logger(), "velodyne");
//                          copyPointCloud(*laserCloudRawWithFeatures, *velodyneLaserCloudRawWithFeatures);
//                      }
//                      else
//                      {
//                          // RCLCPP_WARN(this->get_logger(), "ouster");
//                          copyPointCloud(*laserCloudRawWithFeatures, *ousterLaserCloudRawWithFeatures);
//                      }

//                      for(auto &p : prevCornerCloud) {
//                          Eigen::Vector3d p_curr(p.x, p.y, p.z);
//                          Eigen::Vector3d p_curr_tf = deltaTransform.rot * p_curr + deltaTransform.pos;
//                          p.x = p_curr_tf.x();
//                          p.y = p_curr_tf.y();
//                          p.z = p_curr_tf.z();
//                          laserCloudCornerLast->push_back(std::move(p));
//                      }
//                      for(auto &p : prevSurfCloud) {
//                          Eigen::Vector3d p_curr(p.x, p.y, p.z);
//                          Eigen::Vector3d p_curr_tf = deltaTransform.rot * p_curr + deltaTransform.pos;
//                          p.x = p_curr_tf.x();
//                          p.y = p_curr_tf.y();
//                          p.z = p_curr_tf.z();
//                          laserCloudSurfLast->push_back(std::move(p));
//                      }
//                      if (prevSensorType == SensorType::VELODYNE)
//                      {
//                          RCLCPP_WARN(this->get_logger(), "velodyne");
//                          for (auto &p : prevFullResCloud)
//                          {
//                              Eigen::Vector3d p_curr(p.x, p.y, p.z);
//                              Eigen::Vector3d p_curr_tf = deltaTransform.rot * p_curr + deltaTransform.pos;
//                              p.x = p_curr_tf.x();
//                              p.y = p_curr_tf.y();
//                              p.z = p_curr_tf.z();
//                              velodyneLaserCloudRawWithFeatures->push_back(std::move(p));
//                          }
//                      }else{
//                          RCLCPP_WARN(this->get_logger(), "ouster");

//                          for (auto &p : prevFullResCloud)
//                          {
//                              Eigen::Vector3d p_curr(p.x, p.y, p.z);
//                              Eigen::Vector3d p_curr_tf = deltaTransform.rot * p_curr + deltaTransform.pos;
//                              p.x = p_curr_tf.x();
//                              p.y = p_curr_tf.y();
//                              p.z = p_curr_tf.z();
//                              ousterLaserCloudRawWithFeatures->push_back(std::move(p));
//                          }
//                      }

//                  }

//                  // clear dual scan buffers
//                  while(!cornerDualScanBuf.empty()) cornerDualScanBuf.pop();
//                  while(!surfDualScanBuf.empty()) surfDualScanBuf.pop();
//                  while(!fullResDualScanBuf.empty()) fullResDualScanBuf.pop();
//                  while(!scanTransformBuf.empty()) scanTransformBuf.pop();
//                  while(!sensorTypeBuf.empty()) sensorTypeBuf.pop();

//                  // RCLCPP_WARN(this->get_logger(), "finished converting second scan");
//              }
//              else
//              {
//                  if(sensor_type == SensorType::VELODYNE)
//                  {
//                      copyPointCloud(*laserCloudRawWithFeatures, *velodyneLaserCloudRawWithFeatures);
//                  }else{
//                      RCLCPP_WARN(this->get_logger(), "saving scan 1");
//                      // stack 2 frames (horizontal and vertical) before doing Registration
//                      cornerDualScanBuf.push(*laserCloudCornerLast);
//                      surfDualScanBuf.push(*laserCloudSurfLast);
//                      fullResDualScanBuf.push(*laserCloudRawWithFeatures);
//                      scanTransformBuf.push(current_pose);
//                      sensorTypeBuf.push(sensor_type);
//                      return;
//                  }

//              }

//          // RCLCPP_WARN(this->get_logger(), "velodyne size: %d", (int) velodyneLaserCloudRawWithFeatures->points.size());
//          // RCLCPP_WARN(this->get_logger(), "ouster size: %d", (int) ousterLaserCloudRawWithFeatures->points.size());

//      }

//     void laserMapping::adjustVoxelSize(int &laser_cloud_corner_stack_num, int &laser_cloud_surf_stack_num){

//         // Calculate cloud statistics
//         bool increase_blind_radius = false;
//         if(config_.auto_voxel_size)
//         {
//             Eigen::Vector3f average(0,0,0);
//             int count_far_points = 0;
//             RCLCPP_INFO(this->get_logger(), "Asurface size: %zu", laserCloudSurfLast->points.size());
//             for (auto &point : *laserCloudSurfLast)
//             {
//                 average(0) += fabs(point.x);
//                 average(1) += fabs(point.y);
//                 average(2) += fabs(point.z);
//                 if(point.x*point.x + point.y*point.y + point.z*point.z>9){
//                     count_far_points++;
//                 }
//             }
//             RCLCPP_INFO(this->get_logger(), "count_far_points: %d", count_far_points);
//             if (count_far_points > 2000)
//             {
//                 increase_blind_radius = true;
//             }
//             average /= laserCloudSurfLast->points.size();
//             // RCLCPP_INFO_STREAM(this->get_logger(), "average: " << average);
//             stats.average_distance = average(0)*average(1)*average(2);
//             if (stats.average_distance < 25)
//             {
//                 RCLCPP_WARN(this->get_logger(), "tiny area");
//                 config_.lineRes = 0.1;
//                 config_.planeRes = 0.2;
//             }
//             else if (stats.average_distance > 65)
//             {
//                 RCLCPP_WARN(this->get_logger(), "large area");
//                 config_.lineRes = 0.4;
//                 config_.planeRes = 0.8;
//             }
//             downSizeFilterSurf.setLeafSize(config_.planeRes , config_.planeRes , config_.planeRes );
//             downSizeFilterCorner.setLeafSize(config_.lineRes , config_.lineRes , config_.lineRes );

//         }

//         if(increase_blind_radius)
//         {
//             RCLCPP_WARN(this->get_logger(), "increase blind radius");
//             pcl::CropBox<PointType> boxFilter;
//             float min = -2.0;
//             float max = -min;
//             boxFilter.setMin(Eigen::Vector4f(min, min, min, 1.0));
//             boxFilter.setMax(Eigen::Vector4f(max, max, max, 1.0));
//             boxFilter.setNegative(true);
//             boxFilter.setInputCloud(laserCloudCornerLast);
//             boxFilter.filter(*laserCloudCornerLast);
//             boxFilter.setInputCloud(laserCloudSurfLast);
//             boxFilter.filter(*laserCloudSurfLast);

//             RCLCPP_INFO(this->get_logger(), "surface size: %zu", laserCloudSurfLast->points.size());
//             RCLCPP_INFO(this->get_logger(), "corner size: %zu", laserCloudCornerLast->points.size());
//         }

//         laserCloudCornerStack->clear();
//         downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
//         downSizeFilterCorner.filter(*laserCloudCornerStack);
//         laser_cloud_corner_stack_num = laserCloudCornerStack->points.size();

//         laserCloudSurfStack->clear();
//         downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
//         downSizeFilterSurf.filter(*laserCloudSurfStack);
//         laser_cloud_surf_stack_num = laserCloudSurfStack->points.size();

//         localmap.lineRes_ = config_.lineRes;
//         localmap.planeRes_ = config_.planeRes;

//         // If we have too many features, downsize a little more to avoid too much cpu usage
//         if (laser_cloud_surf_stack_num > 2000)
//         {
//             // RCLCPP_WARN(this->get_logger(), "Too many surface features, downsizing it. before: %d", laser_cloud_surf_stack_num);
//             downSizeFilterSurf.setLeafSize(config_.planeRes * 1.5, config_.planeRes * 1.5, config_.planeRes * 1.5);
//             laserCloudSurfStack->clear();
//             downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
//             downSizeFilterSurf.filter(*laserCloudSurfStack);
//             downSizeFilterSurf.setLeafSize(config_.planeRes, config_.planeRes, config_.planeRes);
//             laser_cloud_surf_stack_num = laserCloudSurfStack->points.size();

//             if(config_.auto_voxel_size)
//             {
//                 // localmap.lineRes_ = config_.lineRes * 2.0;
//                 localmap.planeRes_ = config_.planeRes * 1.5;
//             }
//             // RCLCPP_WARN(this->get_logger(), "surface features final size: %d", laser_cloud_surf_stack_num);
//         }

//     }

//     void laserMapping::publishTopic(Eigen::Vector3i postion_in_locamap ) {

//         TicToc t_pub;
//         // publish surround map for every 5 frame
//         if (frameCount % 5 == 0 && config_.debug_view_enabled)
//         {
//             laserCloudSurround->clear();
//             *laserCloudSurround = localmap.get5x5LocalMap(postion_in_locamap);
//             // RCLCPP_WARN(this->get_logger(), "Cloud surround size %d", laserCloudSurround->points.size());
//             sensor_msgs::msg::PointCloud2 laserCloudSurround3;
//             pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
//             laserCloudSurround3.header.stamp =
//                     rclcpp::Time(timeLaserOdometry);
//             laserCloudSurround3.header.frame_id = config_.WORLD_FRAME;
//             pubLaserCloudSurround->publish(laserCloudSurround3);
//         }

//         // if (frameCount % 20 == 0)
//         // {
//         //     pcl::PointCloud<PointType> laserCloudMap;
//         //     laserCloudMap = localmap.get_all_localmap();
//         //     sensor_msgs::msg::PointCloud2 laserCloudMsg;
//         //     pcl::toROSMsg(laserCloudMap, laserCloudMsg);
//         //     laserCloudMsg.header.stamp = rclcpp::Time(timeLaserOdometry);
//         //     laserCloudMsg.header.frame_id = "sensor_init";
//         //     pubLaserCloudMap->publish(laserCloudMsg);
//         // }

//         // RCLCPP_WARN(this->get_logger(), "velodyne pointcloud size: %d", (int) velodyneLaserCloudRawWithFeatures->points.size());
//         // // Doing everything in one loop now
//         if (localizationCount <= 0)
//         {
//             int laserCloudFullResNum = velodyneLaserCloudRawWithFeatures->points.size();
//             LOG(INFO)<<"velodyneLaserCloudRawWithFeatures num:"<<laserCloudFullResNum;
//             laserCloudFullRes_rot->clear();
//             laserCloudFullRes_rot->resize(laserCloudFullResNum);
//             for (int i = 0; i < laserCloudFullResNum; i++)
//             {
//                 pointAssociateToMap(&velodyneLaserCloudRawWithFeatures->points[i],
//                                     &velodyneLaserCloudRawWithFeatures->points[i]);

//                 laserCloudFullRes_rot->points[i].x = velodyneLaserCloudRawWithFeatures->points[i].y;
//                 laserCloudFullRes_rot->points[i].y = velodyneLaserCloudRawWithFeatures->points[i].z;
//                 laserCloudFullRes_rot->points[i].z = velodyneLaserCloudRawWithFeatures->points[i].x;

//                 // Change frame of raw with features
// //                velodyneLaserCloudRawWithFeatures->points[i].x = laserCloudFullRes_rot->points[i].x;
// //                velodyneLaserCloudRawWithFeatures->points[i].y = laserCloudFullRes_rot->points[i].y;
// //                velodyneLaserCloudRawWithFeatures->points[i].z = laserCloudFullRes_rot->points[i].z;
//                 laserCloudFullRes_rot->points[i].intensity = velodyneLaserCloudRawWithFeatures->points[i].h;
//             }

//             sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
//             pcl::toROSMsg(*velodyneLaserCloudRawWithFeatures, laserCloudFullRes3);
//             laserCloudFullRes3.header.stamp = rclcpp::Time(timeLaserOdometry);
//             laserCloudFullRes3.header.frame_id = config_.WORLD_FRAME;
//             pubLaserCloudFullRes->publish(laserCloudFullRes3);

//             sensor_msgs::msg::PointCloud2 laserCloudFullRes4;
//             pcl::toROSMsg(*laserCloudFullRes_rot, laserCloudFullRes4);
//             laserCloudFullRes4.header.stamp = rclcpp::Time(timeLaserOdometry);
//             laserCloudFullRes4.header.frame_id = config_.WORLD_FRAME_ROT;
//             pubLaserCloudFullRes_rot->publish(laserCloudFullRes4);

//             // ouster
//             laserCloudFullResNum = ousterLaserCloudRawWithFeatures->points.size();
//             if (laserCloudFullResNum > 0)
//             {
//                 laserCloudFullRes_rot->clear();
//                 laserCloudFullRes_rot->resize(laserCloudFullResNum);
//                 for (int i = 0; i < laserCloudFullResNum; i++)
//                 {
//                     pointAssociateToMap(&ousterLaserCloudRawWithFeatures->points[i],
//                                         &ousterLaserCloudRawWithFeatures->points[i]);

//                     laserCloudFullRes_rot->points[i].x = ousterLaserCloudRawWithFeatures->points[i].y;
//                     laserCloudFullRes_rot->points[i].y = ousterLaserCloudRawWithFeatures->points[i].z;
//                     laserCloudFullRes_rot->points[i].z = ousterLaserCloudRawWithFeatures->points[i].x;

//                     // Change frame of raw with features
//                     // ousterLaserCloudRawWithFeatures->points[i].x = laserCloudFullRes_rot->points[i].x;
//                     // ousterLaserCloudRawWithFeatures->points[i].y = laserCloudFullRes_rot->points[i].y;
//                     // ousterLaserCloudRawWithFeatures->points[i].z = laserCloudFullRes_rot->points[i].z;
//                     // laserCloudFullRes_rot->points[i].intensity = ousterLaserCloudRawWithFeatures->points[i].h;
//                 }

//                 // sensor_msgs::msg::PointCloud2 laserCloudFullResOusterWithFeatures;
//                 // pcl::toROSMsg(*ousterLaserCloudRawWithFeatures, laserCloudFullResOusterWithFeatures);
//                 // laserCloudFullResOusterWithFeatures.header.stamp = rclcpp::Time(timeLaserOdometry);
//                 // laserCloudFullResOusterWithFeatures.header.frame_id = "sensor_init_rot";
//                 // pubLaserCloudFullResOusterWithFeatures->publish(laserCloudFullResOusterWithFeatures);

//                 sensor_msgs::msg::PointCloud2 laserCloudFullResOuster;
//                 pcl::toROSMsg(*laserCloudFullRes_rot, laserCloudFullResOuster);
//                 laserCloudFullResOuster.header.stamp = rclcpp::Time(timeLaserOdometry);
//                 laserCloudFullResOuster.header.frame_id = config_.WORLD_FRAME_ROT;
//                 pubLaserCloudFullResOuster->publish(laserCloudFullResOuster);
//             }
//         }
//         else
//         {
//             // if (localizationCount > 0)
//             localizationCount--;
//         }


//         publishOdometry();

//     }

//     void laserMapping::mappingOptimization(Eigen::Vector3i &postion_in_locamap, Transformd start_tf,tf2::Quaternion roll_pitch_quat,
//             const int laser_cloud_corner_stack_num, const int laser_cloud_surf_stack_num)
//     {

//         if (initialization == true)
//         {

//             postion_in_locamap = localmap.shiftMap(t_w_curr);

//             TicToc t_shift;

//             auto [laser_cloud_corner_from_map_num, laser_cloud_surf_from_map_num] =
//             localmap.get5x5LocalMapFeatureSize(postion_in_locamap);

//             // printf("map prepare time %f ms\n", t_shift.toc());
//             // RCLCPP_INFO(this->get_logger(), "map corner num %d  surf num %d", laser_cloud_corner_from_map_num,
//             //  laser_cloud_surf_from_map_num);
//             stats.laser_cloud_corner_from_map_num = laser_cloud_corner_from_map_num;
//             stats.laser_cloud_surf_from_map_num = laser_cloud_surf_from_map_num;
//             stats.laser_cloud_corner_stack_num = laser_cloud_corner_stack_num;
//             stats.laser_cloud_surf_stack_num = laser_cloud_surf_stack_num;
//             stats.iterations.clear();

//             if (config_.debug_view_enabled)
//             {
//                 debug_view->addOptimizationIterationPose(T_w_lidar, timeLaserOdometry);
//                 debug_view->publishFeaturesFromScan(*laserCloudCornerStack, *laserCloudSurfStack,
//                                                     timeLaserOdometry);
//                 debug_view->publishFeaturesFromMap(localmap.get_5x5_localmap_corner(postion_in_locamap),
//                                                    localmap.get_5x5_localmap_surf(postion_in_locamap),
//                                                    timeLaserOdometry);
//                 debug_view->clearCorrespondences();
//             }



//             if (laser_cloud_corner_from_map_num > 10 && laser_cloud_surf_from_map_num > 50)
//             {
//                 TicToc t_opt;
//                 // TODO: improve optimization time = 26ms !
//                 for (int iterCount = 0; iterCount < config_.max_iterations; iterCount++)
//                 {
//                     super_odometry_msgs::msg::IterationStats iter_stats;

//                     // ceres::LossFunction *loss_function = NULL;
//                     ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

//                     ceres::Problem::Options problem_options;

//                     ceres::Problem problem(problem_options);

//                     if (AUTO_JACOBIAN)
//                     {
//                         ceres::LocalParameterization *q_parameterization =
//                                 new ceres::EigenQuaternionParameterization();
//                         problem.AddParameterBlock(parameters, 4, q_parameterization);
//                         problem.AddParameterBlock(parameters + 4, 3);
//                     }
//                     else
//                     {
//                         problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
//                     }

//                     TicToc t_data;
//                     int corner_num = 0;

//                     if(config_.debug_view_enabled)
//                         debug_view->clearCorrespondences();
//                     for (int i = 0; i < laser_cloud_corner_stack_num; i++)
//                     {
//                         pointOri = laserCloudCornerStack->points[i];
//                         // double sqrtDis = pointOri.x * pointOri.x + pointOri.y *
//                         // pointOri.y + pointOri.z * pointOri.z;
//                         pointAssociateToMap(&pointOri, &pointSel);

//                         std::vector<PointType> k_pts;
//                         std::vector<float> pointSearchSqDis;
//                         auto status = localmap.nearestKSearchEdgePoint(pointSel, k_pts,
//                                                                      pointSearchSqDis);

//                         if (k_pts.size() < 5 or pointSearchSqDis.size() < 5)
//                             status = false;

//                         if (status)
//                         {
//                             if (pointSearchSqDis.at(4) < 3*localmap.lineRes_)
//                             {
//                                 std::vector<Eigen::Vector3d> nearCorners;
//                                 Eigen::Vector3d center(0, 0, 0);
//                                 for (int j = 0; j < 5; j++)
//                                 {

//                                     Eigen::Vector3d tmp(k_pts.at(j).x, k_pts.at(j).y, k_pts.at(j).z);
//                                     center = center + tmp;
//                                     nearCorners.push_back(tmp);
//                                 }
//                                 center = center / 5.0;

//                                 Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
//                                 for (int j = 0; j < 5; j++)
//                                 {
//                                     Eigen::Matrix<double, 3, 1> tmpZeroMean =
//                                             nearCorners.at(j) - center;
//                                     covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
//                                 }

//                                 Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

//                                 // if is indeed line featuresuperodometry_ueye_2020-09-16-19-51-38_0.bag

//                                 // note Eigen library sort eigenvalues in increasing order
//                                 Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
//                                 Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
//                                 if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
//                                 {
//                                     Eigen::Vector3d point_on_line = center;
//                                     Eigen::Vector3d point_a, point_b;
//                                     point_a = 0.1 * unit_direction + point_on_line;
//                                     point_b = -0.1 * unit_direction + point_on_line;

//                                     if (AUTO_JACOBIAN)
//                                     {
//                                         ceres::CostFunction *cost_function = LidarEdgeFactor::Create(
//                                                 curr_point,
//                                                 point_a,
//                                                 point_b,
//                                                 1.0);
//                                         problem.AddResidualBlock(cost_function, loss_function, parameters,
//                                                                  parameters + 4);
//                                     }
//                                     else
//                                     {
//                                         ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(
//                                                 curr_point,
//                                                 point_a, point_b);
//                                         problem.AddResidualBlock(cost_function, loss_function, parameters);
//                                     }

//                                     corner_num++;
//                                 }
//                             }
//                         }
//                     }

//                     double sampling_rate = -1.0;
//                     if(laser_cloud_surf_stack_num > config_.max_surface_features)
//                         sampling_rate = 1.0*config_.max_surface_features/laser_cloud_surf_stack_num;


//                     int surf_num = 0;
//                     // int keep = 0;
//                     // int reject_num_neigbors = 0;
//                     // int reject_plane_fit = 0;
//                     // int reject_nn_search = 0;
//                     // int reject_search_radius = 0;
//                     // laser_cloud_surf_stack_num = 0;
//                     for (int i = 0; i < laser_cloud_surf_stack_num; i++)
//                     {
//                         if(sampling_rate>0.0)
//                         {
//                             double remainder = fmod(i*sampling_rate, 1.0);
//                             if (remainder + 0.001 > sampling_rate)
//                                 continue;
//                         }
//                         // keep++;

//                         pointOri = laserCloudSurfStack->points[i];
//                         // double sqrtDis = pointOri.x * pointOri.x + pointOri.y *
//                         // pointOri.y + pointOri.z * pointOri.z;
//                         pointAssociateToMap(&pointOri, &pointSel);

//                         std::vector<PointType> k_pts;
//                         std::vector<float> pointSearchSqDis;
//                         auto status =
//                                 localmap.nearestKSearchSurf(pointSel, k_pts, pointSearchSqDis,5);




//                         if (k_pts.size() < 5 or pointSearchSqDis.size() < 5)
//                         {
//                             // reject_num_neigbors++;
//                             continue;
//                         }

//                         Eigen::Matrix<double, 5, 3> matA0;
//                         Eigen::Matrix<double, 5, 1> matB0 =
//                                 -1 * Eigen::Matrix<double, 5, 1>::Ones();
//                         if (status)
//                         {
//                             if (pointSearchSqDis.at(4) < 3*localmap.planeRes_)
//                             {
//                                 for (int j = 0; j < 5; j++)
//                                 {
//                                     matA0(j, 0) = k_pts.at(j).x;
//                                     matA0(j, 1) = k_pts.at(j).y;
//                                     matA0(j, 2) = k_pts.at(j).z;
//                                     // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
//                                 }
//                                 // find the norm of plane
//                                 Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
//                                 double negative_OA_dot_norm = 1 / norm.norm();
//                                 norm.normalize();

//                                 // Here n(pa, pb, pc) is unit norm of plane
//                                 bool planeValid = true;
//                                 for (int j = 0; j < 5; j++)
//                                 {
//                                     // if OX * n > 0.2, then plane is not fit well
//                                     if (fabs(norm(0) * k_pts.at(j).x + norm(1) * k_pts.at(j).y +
//                                              norm(2) * k_pts.at(j).z + negative_OA_dot_norm) > localmap.planeRes_/2.0)
//                                     {
//                                         planeValid = false;
//                                         // reject_plane_fit++;
//                                         break;
//                                     }
//                                 }
//                                 Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);



//                                 if (planeValid)
//                                 {

//                                     if (AUTO_JACOBIAN)
//                                     {
//                                         ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(
//                                                 curr_point, norm,
//                                                 negative_OA_dot_norm);
//                                         problem.AddResidualBlock(cost_function, loss_function, parameters,
//                                                                  parameters + 4);
//                                     }
//                                     else
//                                     {
//                                         ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(
//                                                 curr_point,
//                                                 norm,
//                                                 negative_OA_dot_norm);
//                                         problem.AddResidualBlock(cost_function, loss_function, parameters);
//                                     }
//                                     if (config_.debug_view_enabled)
//                                     {
//                                         Eigen::Vector3d viewpoint_direction = T_w_lidar.pos - curr_point;
//                                         Eigen::Vector3d debug_norm = norm;

//                                         double dot_product = viewpoint_direction.dot(norm);
//                                         if (dot_product < 0)
//                                         {
//                                             debug_norm = -norm;
//                                         }
//                                         debug_view->addCorrespondences(k_pts, pointSel, false, debug_norm);
//                                     } // orient the normals for visualization

//                                     surf_num++;
//                                 }
//                             }
//                             else
//                             {
//                                 // reject_search_radius++;
//                             }
//                         }else{
//                             // reject_nn_search++;
//                         }
//                     }

//                     iter_stats.num_surf_from_scan = surf_num;
//                     iter_stats.num_corner_from_scan = corner_num;
//                     // RCLCPP_WARN(this->get_logger(), "keep: %d", keep);
//                     // RCLCPP_WARN(this->get_logger(), "reject_plane_fit: %d", reject_plane_fit);
//                     // RCLCPP_WARN(this->get_logger(), "reject_num_neigbors: %d", reject_num_neigbors);
//                     // RCLCPP_WARN(this->get_logger(), "reject_nn_search: %d", reject_nn_search);
//                     // RCLCPP_WARN(this->get_logger(), "reject_search_radius: %d", reject_search_radius);
//                     // RCLCPP_WARN(this->get_logger(), "surf_num: %d", surf_num);
// #if 0
//                     // add absolute constraints
//                             // if (frameCount == 10)
//                             // {
//                                 Eigen::Mat66d information;
//                                 information.setIdentity();
//                                 information(0, 0) = 100.0;
//                                 information(1, 1) = 100.0;
//                                 information(2, 2) = 0;

//                                 information(3, 3) = 0.0;
//                                 information(4, 4) = 0.0;
//                                 information(5, 5) = 0.0;

//                                 // if (laser_imu_sync == true)
//                                 // {
//                                     RCLCPP_INFO(this->get_logger(), "\033[1;32m----> ADD Absolute factor in lasermapping.\033[0m");
//                                     SE3AbsolutatePoseFactor *absolutatePoseFactor =
//                                         new SE3AbsolutatePoseFactor(T_w_lidar, information);
//                                     problem.AddResidualBlock(absolutatePoseFactor, nullptr, parameters);
//                                     // }
//                             // }

// #endif

//                     // printf("corner num %d used corner num %d \n", laser_cloud_corner_stack_num, corner_num);
//                     // printf("surf num %d used surf num %d \n", laser_cloud_surf_stack_num, surf_num);

//                     // printf("mapping data assosiation time %f ms \n", t_data.toc());

//                     // problem.SetParameterLowerBound(parameters, 4, T_w_lidar.pos.x() - 0.1);
//                     // problem.SetParameterLowerBound(parameters, 5, T_w_lidar.pos.y() - 0.1);
//                     // problem.SetParameterLowerBound(parameters, 6, T_w_lidar.pos.z() - 0.1);
//                     // problem.SetParameterUpperBound(parameters, 4, T_w_lidar.pos.x() + 0.1);
//                     // problem.SetParameterUpperBound(parameters, 5, T_w_lidar.pos.y() + 0.1);
//                     // problem.SetParameterUpperBound(parameters, 6, T_w_lidar.pos.z() + 0.1);

//                     TicToc t_solver;
//                     ceres::Solver::Options options;
//                     options.linear_solver_type = ceres::DENSE_QR;
//                     options.max_num_iterations = 4;
//                     options.minimizer_progress_to_stdout = false;
//                     options.check_gradients = false;
//                     options.gradient_check_relative_precision = 1e-4;
//                     ceres::Solver::Summary summary;
//                     ceres::Solve(options, &problem, &summary);
//                     //    printf("mapping solver time %f ms \n", t_solver.toc());
//                     Transformd previous_T(T_w_lidar);
//                     T_w_lidar.rot = q_w_curr;
//                     T_w_lidar.pos = t_w_curr;

//                     if (use_imu_roll_pitch_this_step)
//                     {
//                         double roll, pitch, yaw;
//                         tf2::Quaternion orientation;
//                         tf2::Quaternion orientation_curr(T_w_lidar.rot.x(), T_w_lidar.rot.y(), T_w_lidar.rot.z(), T_w_lidar.rot.w());
//                         tf2::Matrix3x3(orientation_curr).getRPY(roll, pitch, yaw);
//                         // RCLCPP_INFO(this->get_logger(), "yaw: %f", yaw);
//                         tf2::Quaternion yaw_quat;
//                         yaw_quat.setRPY(0, 0, yaw);

//                         orientation = yaw_quat * roll_pitch_quat;
//                         q_w_curr = Eigen::Quaterniond(orientation.w(), orientation.x(), orientation.y(), orientation.z());
//                         T_w_lidar.rot = q_w_curr;
//                     }

//                     // RCLCPP_INFO_STREAM(this->get_logger(), "T_w_lidar: " << T_w_lidar);
//                     // RCLCPP_INFO_STREAM(this->get_logger(), "previous_T: " << previous_T);

//                     Transformd incremental_T = previous_T.inverse() * T_w_lidar;
//                     // RCLCPP_INFO_STREAM(this->get_logger(), "incremental_T: " << incremental_T);
//                     iter_stats.translation_norm = incremental_T.pos.norm();
//                     iter_stats.rotation_norm =
//                             2 * atan2(incremental_T.rot.vec().norm(), incremental_T.rot.w());
//                     // printf("time %f \n", timeLaserOdometry);
//                     // printf("corner factor num %d surf factor num %d\n", corner_num,
//                     // surf_num); printf("result q %f %f %f %f result t %f %f %f\n",
//                     // parameters[3], parameters[0], parameters[1], parameters[2],
//                     //	   parameters[4], parameters[5], parameters[6]);
//                     stats.iterations.push_back(iter_stats);

//                     if (config_.debug_view_enabled)
//                     {
//                         debug_view->addOptimizationIterationPose(T_w_lidar, timeLaserOdometry);
//                         debug_view->publishCorrespondences(timeLaserOdometry);
//                     }

//                     // if (iter_stats.translation_norm < 0.001)
//                     //     break;
//                 }
//                 double time_elapsed = t_opt.toc();
//                 stats.time_elapsed = time_elapsed;
//                 // printf("mapping optimization time %f \n", time_elapsed);
//             }
//             else
//             {
//                 RCLCPP_WARN(this->get_logger(), "time Map corner and surf num are not enough");
//             }
//         }

//         // RCLCPP_INFO_STREAM(this->get_logger(), "Start TF: " << start_tf);
//         // RCLCPP_INFO_STREAM(this->get_logger(), "FINAL TF: " << T_w_lidar);
//         // RCLCPP_INFO_STREAM(this->get_logger(), "last_T_w_lidar: " << last_T_w_lidar);
//         double roll, pitch, yaw;
//         tf2::Quaternion orientation2(T_w_lidar.rot.x(), T_w_lidar.rot.y(), T_w_lidar.rot.z(), T_w_lidar.rot.w());
//         tf2::Matrix3x3(orientation2).getRPY(roll, pitch, yaw);
//         // RCLCPP_INFO(this->get_logger(), "aft mapped orientation: %f %f %f", roll, pitch, yaw);

//         total_incremental_T = start_tf.inverse() * T_w_lidar;

//         stats.total_translation = (total_incremental_T).pos.norm();
//         // RCLCPP_INFO_STREAM(this->get_logger(), "total_translation" << stats.total_translation);
//         // RCLCPP_INFO_STREAM(this->get_logger(), "total_rotation" << 2 * atan2(total_incremental_T.rot.vec().norm(), total_incremental_T.rot.w()));
//         stats.total_rotation = 2 * atan2(total_incremental_T.rot.vec().norm(), total_incremental_T.rot.w());

//         Transformd diff_from_last_T = last_T_w_lidar.inverse() * T_w_lidar;
//         stats.translation_from_last = diff_from_last_T.pos.norm();
//         stats.rotation_from_last = 2 * atan2(diff_from_last_T.rot.vec().norm(), diff_from_last_T.rot.w());

//         bool acceptResult = true;

//         double delta_t = timeLaserOdometry - timeLastMappingResult.toSec();
//         // RCLCPP_INFO(this->get_logger(), "delta_t: %f", delta_t);
//         if (stats.translation_from_last/delta_t > config_.velocity_failure_threshold)
//         {
//             T_w_lidar = last_T_w_lidar;
//             startupCount = 5;
//             acceptResult = false;
//             RCLCPP_WARN(this->get_logger(), "translation from last: %f", stats.translation_from_last);
//             RCLCPP_WARN(this->get_logger(), "large motion detected, ignoring predictor for a while");
//         }

//         // Try to not accumulate noise when being still
//         if ( (stats.translation_from_last < 0.02 && stats.rotation_from_last < 0.005) && stats.laser_cloud_corner_from_map_num > 10 && stats.laser_cloud_surf_from_map_num > 50)
//         {
//             acceptResult = false;
//             if(stats.translation_from_last < 0.005 && stats.rotation_from_last < 0.005)
//             {
//                 T_w_lidar = last_T_w_lidar;
//             }
//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10.0, 
//                                 "very small motion, not accumulating. %f", stats.translation_from_last);
//         }

//         last_T_w_lidar = T_w_lidar;
//         q_w_curr = T_w_lidar.rot;
//         t_w_curr = T_w_lidar.pos;


//         TicToc t_add_and_filter;

//         // Use the corner and surface points for the full resolution pointcloud
//         // int laserCloudFullResNum = laserCloudFullRes->points.size();

//         // TODO: acclerate add point time = 16ms !

//         // laser_cloud_corner_stack_num = laserCloudCornerStack->points.size();
//         // laser_cloud_surf_stack_num = laserCloudSurfStack->points.size();

//         // if (config_.publish_only_feature_points) {
//         //     int laserCloudFullResNum = laser_cloud_corner_stack_num + laser_cloud_surf_stack_num;
//         //     laserCloudFullRes_rot->clear();
//         //     laserCloudFullRes_rot->resize(laserCloudFullResNum);

//         //     for (int i = 0; i < laser_cloud_corner_stack_num; i++) {
//         //         pointAssociateToMap(&laserCloudCornerStack->points[i],
//         //                             &laserCloudCornerStack->points[i]);

//         //         laserCloudFullRes_rot->points[i].x = laserCloudCornerStack->points[i].y;
//         //         laserCloudFullRes_rot->points[i].y = laserCloudCornerStack->points[i].z;
//         //         laserCloudFullRes_rot->points[i].z = laserCloudCornerStack->points[i].x;
//         //         laserCloudFullRes_rot->points[i].intensity = laserCloudCornerStack->points[i].intensity;
//         //     }

//         //     int full_res_index = laser_cloud_corner_stack_num;
//         //     for (int i = 0; i < laser_cloud_surf_stack_num; i++) {
//         //         pointAssociateToMap(&laserCloudSurfStack->points[i],
//         //                             &laserCloudSurfStack->points[i]);

//         //         laserCloudFullRes_rot->points[full_res_index].x = laserCloudSurfStack->points[i].y;
//         //         laserCloudFullRes_rot->points[full_res_index].y = laserCloudSurfStack->points[i].z;
//         //         laserCloudFullRes_rot->points[full_res_index].z = laserCloudSurfStack->points[i].x;
//         //         laserCloudFullRes_rot->points[full_res_index].intensity = laserCloudSurfStack->points[i].intensity;
//         //         full_res_index++;
//         //     }


//         for (int i = 0; i < laser_cloud_corner_stack_num; i++)
//         {
//             pointAssociateToMap(&laserCloudCornerStack->points[i],
//                                 &laserCloudCornerStack->points[i]);
//         }

//         for (int i = 0; i < laser_cloud_surf_stack_num; i++)
//         {
//             pointAssociateToMap(&laserCloudSurfStack->points[i],
//                                 &laserCloudSurfStack->points[i]);
//         }



//         // RCLCPP_INFO(this->get_logger(), "Localization count %d", localizationCount);

//         if (localizationCount <= 0 and acceptResult)
//         {
//             // RCLCPP_INFO(this->get_logger(), "adding points to the map");
//             localmap.addEdgePointCloud(*laserCloudCornerStack);
//             localmap.addSurfPointCloud(*laserCloudSurfStack);
//         }

//         //   printf("add points and filter time %f ms\n", t_add_and_filter.toc());

//     }




// //#pragma clang diagnostic push
// //#pragma clang diagnostic ignored "-Wmissing-noreturn"

//     void laserMapping::process()
//     {
//         rclcpp::Rate r(100.0);
//         while (ros::ok())
//         {
//             r.sleep();
//             ros::spinOnce();
//             // if (!got_previous_map && config_.start_from_previous_map)
//             // {
//             //     RCLCPP_INFO_THROTTLE(this->get_logger(), 0.2, "waiting for previous map");
//             //     continue;
//             // }

// #if DEBUG_VIEW
//             RCLCPP_INFO(this->get_logger(), "time corner %f surf %f full %f odom %f \n",
//                      timeLaserCloudCornerLast, timeLaserCloudSurfLast,
//                      timeLaserCloudFullRes, timeLaserOdometry);

//             RCLCPP_INFO(this->get_logger(), "Buffer sizes");
//             RCLCPP_INFO(this->get_logger(), "cornerLastBuf %d", cosrnerLastBuf.size());
//             RCLCPP_INFO(this->get_logger(), "surfLastBuf %d", surfLastBuf.size());
//             RCLCPP_INFO(this->get_logger(), "fullResBuf %d", fullResBuf.size());
//             RCLCPP_INFO(this->get_logger(), "rawDataBuf %d", rawDataBuf.size());
// #endif

//             while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
//                    !rawWithFeaturesBuf.empty() && !IMUPredictionBuf.empty() && !sensorTypeLastBuf.empty())
//             {

//                 imuorientationAvailable = true;
//                 mBuf.lock();

//                 timeLaserCloudCornerLast = cornerLastBuf.front().header.stamp.seconds();
//                 timeLaserCloudSurfLast = surfLastBuf.front().header.stamp.seconds();
//                 timeLaserCloudFullRes = rawWithFeaturesBuf.front().header.stamp.seconds();
//                 timeLaserOdometry = timeLaserCloudFullRes;

//                 if (abs(timeLaserCloudCornerLast - timeLaserOdometry) > 0.0 ||
//                     abs(timeLaserCloudSurfLast - timeLaserOdometry) > 0.0 ||
//                     abs(timeLaserCloudFullRes - timeLaserOdometry) > 0.0)
//                 {

//                     // RCLCPP_INFO(this->get_logger(), "time corner %f surf %f full %f odom %f \n",
//                     //  timeLaserCloudCornerLast, timeLaserCloudSurfLast,
//                     //  timeLaserCloudFullRes, timeLaserOdometry);
//                     // RCLCPP_INFO(this->get_logger(), "unsync messeage!");
//                     mBuf.unlock();
//                     break;
//                 }
//                 // RCLCPP_INFO(this->get_logger(), "start processing clouds with timestamp: %f", timeLaserOdometry);
//                 // RCLCPP_INFO_STREAM(this->get_logger(), "at time: " << rclcpp::Clock().now();

//                 laserCloudCornerLast->clear();
//                 pcl::fromROSMsg(cornerLastBuf.front(), *laserCloudCornerLast);
//                 cornerLastBuf.pop();

//                 laserCloudSurfLast->clear();
//                 pcl::fromROSMsg(surfLastBuf.front(), *laserCloudSurfLast);
//                 surfLastBuf.pop(); 

//                 if(!realsenseBuf.empty())
//                 { 
//                     laserCloudRealsense->clear();
//                     pcl::fromROSMsg(realsenseBuf.front(),*laserCloudRealsense);
//                     realsenseBuf.pop();
//                 }  

//                 RCLCPP_INFO(this->get_logger(), "laserCloudRealsense size: %zu , realsenseBuf size: %zu", laserCloudRealsense->size(),realsenseBuf.size());

//                 // laserCloudFullRes->clear();
//                 // pcl::fromROSMsg(fullResBuf.front(), *laserCloudFullRes);
//                 // fullResBuf.pop();

//                 laserCloudRawWithFeatures->clear();
//                 pcl::fromROSMsg(rawWithFeaturesBuf.front(), *laserCloudRawWithFeatures);
//                 rawWithFeaturesBuf.pop();

//                 Eigen::Quaterniond IMUPrediction;
//                 IMUPrediction = IMUPredictionBuf.front();
//                 IMUPredictionBuf.pop();

//                 SensorType sensor_type = sensorTypeLastBuf.front();
//                 sensorTypeLastBuf.pop();
//                 // RCLCPP_INFO(this->get_logger(), "sensor type: %d", (int) sensor_type);

//                 IMUPrediction.normalize();

//                 q_wodom_curr.x() = IMUPrediction.x();
//                 q_wodom_curr.y() = IMUPrediction.y();
//                 q_wodom_curr.z() = IMUPrediction.z();
//                 q_wodom_curr.w() = IMUPrediction.w();

//                 RCLCPP_INFO_STREAM(this->get_logger(), "INITIALIZATION3: " << initialization);
//                 setInitialGuess();
//                 Transformd start_tf(T_w_lidar);

//                 while (!cornerLastBuf.empty())
//                 {

//                     // RCLCPP_INFO_STREAM(this->get_logger(), "cornerLastBuf:" << cornerLastBuf.size());
//                     cornerLastBuf.pop();
//                 }

//                 while (!surfLastBuf.empty())
//                 {

//                     // RCLCPP_INFO_STREAM(this->get_logger(), "surfLastBuf:" << surfLastBuf.size());
//                     surfLastBuf.pop();
//                 }
                
//                 while (!realsenseBuf.empty())
//                 {

//                     // RCLCPP_INFO_STREAM(this->get_logger(), "fullResBuf:" << fullResBuf.size());
//                     realsenseBuf.pop();
//                 }

//                 while (!rawWithFeaturesBuf.empty())
//                 {

//                     // RCLCPP_INFO_STREAM(this->get_logger(), "fullResBuf:" << fullResBuf.size());
//                     rawWithFeaturesBuf.pop();
//                 }
             
//                 while (!IMUPredictionBuf.empty())
//                 {

//                     // RCLCPP_INFO_STREAM(this->get_logger(), "IMUPredictionBuf:" << IMUPredictionBuf.size());
//                     IMUPredictionBuf.pop();
//                 }

//                 while (!sensorTypeLastBuf.empty())
//                 {

//                     // RCLCPP_INFO_STREAM(this->get_logger(), "IMUPredictionBuf:" << IMUPredictionBuf.size());
//                     sensorTypeLastBuf.pop();
//                 }


//                 RCLCPP_INFO_STREAM(this->get_logger(), "INITIALIZATION2: " << initialization);

//                 // RCLCPP_INFO_STREAM(this->get_logger(), "T_w_lidar:" << T_w_lidar.pos);
//                 mBuf.unlock();

//                 if(config_.enable_ouster_data)
//                     preprocessDualLidarFeatures(start_tf,sensor_type);

//                 TicToc t_whole;
//                 tf2::Quaternion roll_pitch_quat;
//                 // RCLCPP_INFO(this->get_logger(), "use_imu_roll_pitch: %d", config_.use_imu_roll_pitch);
//                 if (use_imu_roll_pitch_this_step)
//                 {
//                     double start_roll, start_pitch, start_yaw;
//                     tf2::Quaternion orientation(T_w_lidar.rot.x(), T_w_lidar.rot.y(), T_w_lidar.rot.z(), T_w_lidar.rot.w());
//                     tf2::Matrix3x3(orientation).getRPY(start_roll, start_pitch, start_yaw);
//                     // RCLCPP_INFO(this->get_logger(), "Start orientation: %f %f %f", start_roll, start_pitch, start_yaw);
//                     roll_pitch_quat.setRPY(start_roll, start_pitch, 0);
//                 }

//                 int laser_cloud_corner_stack_num=0;
//                 int laser_cloud_surf_stack_num=0;

//                 adjustVoxelSize(laser_cloud_corner_stack_num, laser_cloud_surf_stack_num);

//                 RCLCPP_INFO_STREAM(this->get_logger(), "INITIALIZATION1: " << initialization);
//                 RCLCPP_DEBUG(this->get_logger(), "before surface features  size: %zu", laserCloudSurfStack->size());
//                 if (!laserCloudRealsense->empty())
//                 {
//                     *laserCloudSurfStack = *laserCloudSurfStack + *laserCloudRealsense;
//                     laser_cloud_surf_stack_num = laserCloudSurfStack->points.size();
//                 }

//                 RCLCPP_DEBUG(this->get_logger(), "extra surface features  size: %zu", laserCloudRealsense->size());
//                 RCLCPP_DEBUG(this->get_logger(), "after surface features  size: %zu", laserCloudSurfStack->size());


//                 Eigen::Vector3i postion_in_locamap;

//                 RCLCPP_INFO_STREAM(this->get_logger(), "INITIALIZATION0: " << initialization);

// //                mappingOptimization(postion_in_locamap, start_tf,roll_pitch_quat,
// //                laser_cloud_corner_stack_num, laser_cloud_surf_stack_num);


// //           slam.Localization(initialization, LidarSLAM::PredictionSource::IMU_ODOM, T_w_lidar,
// //                        laserCloudCornerStack, laserCloudSurfStack,timeLaserOdometry,startupCount,stats);

//              slam.Localization(initialization, LidarSLAM::PredictionSource::IMU_ODOM, T_w_lidar,
//                                 laserCloudCornerStack, laserCloudSurfStack);

//               q_w_curr = slam.T_w_lidar.rot;
//               t_w_curr = slam.T_w_lidar.pos;

//               auto euler = q_w_curr.toRotationMatrix().eulerAngles(0, 1, 2);
//               LOG(INFO)<<"position: "<<slam.T_w_lidar.pos;
//               LOG(INFO) << "Euler from quaternion in roll, pitch, yaw: "<< euler;


//               initialization = true;


//               publishTopic(postion_in_locamap);

//               velodyneLaserCloudRawWithFeatures->clear();
//               ousterLaserCloudRawWithFeatures->clear();

//                 imuorientationAvailable = false;
//                 frameCount++;
//             }
//             std::chrono::milliseconds dura(2);
//             std::this_thread::sleep_for(dura);
//         }
//     }

//     // #pragma clang diagnostic pop

// } // namespace superodom

// int main(int argc, char **argv) {

//     rclcpp::init(argc,argv);
//     auto node = std::make_shared<superodom::laserMapping>();
//     RCLCPP_INFO(node->get_logger(), "\033[1;32m----> LaserMapping Started.\033[0m");
    
//     rclcpp::spin(node);
//     rclcpp::shutdown();

//     return 0;
// }
