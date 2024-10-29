#pragma once
#include "../feature_manager.h"

#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include "../utility/Twist.h"
#include "../Utils/EigenTypes.h"
#include "../Frontend/frontend_data.h"
#include <pcl/point_cloud.h>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include <iostream>
#include <map>


using namespace Eigen;
using namespace std;
#define USE_DEPTH_INITIAL 0
class ImageFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImageFrame()
    {};
    ImageFrame(const vins::FeatureTrackerResulst &_points,
               double _t)
        : points(_points), timestamp{_t}
    {}

    ImageFrame(const vins::FrontEndResult &input)
    {
        this->points = input.feature;
        this->cloud_ptr = input.cloud_ptr;
        this->image = input.image;
        this->Tw_imu_meas = input.Tw_imu_meas;
        this->vel_imu_meas=input.vel_imu_meas;
        this->Ba_meas=input.Ba_meas;
        this->Bg_meas=input.Bg_meas;
        this->reset_id=input.reset_id;
        this->gravity_meas=input.gravity_meas;
        this->laser_odom_vio_sync=input.laser_odom_vio_sync;
        timestamp = input.timestamp;
    }

public:
    
    vins::FeatureTrackerResulst points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    cv::Mat image;
    IntegrationBase *pre_integration = nullptr;
    
    //laser odometry information
    Transformd Tw_imu_meas;
    Vector3d vel_imu_meas;
    Vector3d Ba_meas;
    Vector3d Bg_meas;
    double gravity_meas;
    int reset_id = -1;  // to notifiy the status of laser odometry

    Transformd Twi;
    double timestamp;
    bool is_key_frame = false;
    bool laser_odom_vio_sync=false;
}; // class ImageFrame


bool
VisualIMUAlignment(Eigen::aligned_map<double, ImageFrame> &all_image_frame, Vector3d *Bgs, Vector3d &g, VectorXd &x);
