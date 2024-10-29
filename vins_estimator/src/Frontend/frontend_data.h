//
// Created by ubuntu on 2020/5/27.
//

#ifndef FRONTEND_DATA_H
#define FRONTEND_DATA_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../Utils/EigenTypes.h"
#include "../utility/Twist.h"
namespace vins
{
using FeatureID = int;
using TimeFrameId = double;
using FeatureTrackerResulst = Eigen::aligned_map<int,
                                                 Eigen::aligned_vector<std::pair<int, Eigen::Matrix<double, 8, 1>>>>;


struct FrontEndResult
{
    typedef std::shared_ptr<FrontEndResult> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // optical flow result
    double timestamp;
    FeatureTrackerResulst feature;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    cv::Mat image;

    //laser odometry measurement 
    Transformd Tw_imu_meas;
    Eigen::Vector3d vel_imu_meas;
    Eigen::Vector3d Ba_meas;
    Eigen::Vector3d Bg_meas;
    double gravity_meas;
    int reset_id;  // to notifiy the status of laser odometry
    bool laser_odom_vio_sync= false;
};

} // namespce vins

#endif //FRONTEND_DATA_H
