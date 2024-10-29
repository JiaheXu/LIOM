//
// Created by ubuntu on 2020/5/26.
//

#ifndef FRONTEND_H
#define FRONTEND_H

#include <mutex>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "frontend_data.h"
#include "../featureTrack/feature_tracker.h"
#include "../parameters.h"


typedef pcl::PointXYZ PointType;
#define SINGLEDEPTH 0 
#define USE_LIDAT_DEPTH 1
#define fuse_global_point 1
class frontend
{
public:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msgImage, sensor_msgs::msg::PointCloud2> syncPolicy;
public:

    explicit frontend();
    static constexpr int N_SCAN = 64;
    static constexpr int Horizon_SCAN = 1000;
    static constexpr float ang_res_x = 0.2;
    static constexpr float ang_res_y = 0.427;
    static constexpr float ang_bottom = 24.9;
    static constexpr int groundScanInd = 16;
    static constexpr bool distortion_flag = false;
    static constexpr double distortion_img[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    static constexpr int Boundary = 4;
    static constexpr int num_bins = 360;

public:

    void
    setUpROS(ros::NodeHandle *pub_node, ros::NodeHandle *private_node);

    void
    img_callback(const sensor_msgs::msgImageConstPtr &color_msg, const sensor_msgs::msg::PointCloud2::SharedPtr lasercloud_msg);


    void
    lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg);
    
    void
    img_callback(const sensor_msgs::msgImageConstPtr &color_msg);

    inline void
    assign_feature_depth(cv::Point2f &p, double &depth);

    static Eigen::Vector2f
    xyz_to_uv(pcl::PointXYZ &xyz);

    Eigen::Vector2f
    xyz_to_uv(const Eigen::Vector3f &xyz);

   static bool
    is_in_image(const Eigen::Vector2d &uv, int boundary, float scale);

    void
    show_image_with_points(cv::Mat &img, size_t num_level);

    void
    cloud_in_image(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudInput);

    void
    get_depth(const geometry_msgs::msg::Point32 &features_2d, pcl::PointXYZI &p);


    bool 
    create_sphere_cloud_kdtree(const rclcpp::Time &stamp_cur,cv::Mat cur_image, const pcl::PointCloud<PointType>::Ptr& depth_cloud);

    void 
    project_cloud_image(const rclcpp::Time &stamp_cur, cv::Mat imageCur, const pcl::PointCloud<PointType>::Ptr &depth_cloud_local);
    
    float 
    pointDistance(PointType p);
    
    float
    pointDistance(PointType p1, PointType p2);

    template<typename T>
    sensor_msgs::msg::PointCloud2 
    publishCloud(ros::Publisher *thisPub, T thisCloud, rclcpp::Time thisStamp, std::string thisFrame);

    void 
    getColor(float p, float np, float&r, float&g, float&b);



public:
    ros::NodeHandle *pub_node_;
    ros::NodeHandle *private_node_;

    ros::Publisher pub_img, pub_match, pub_featuredepth,pub_depthimage;
    ros::Publisher pub_restart,pub_lidar_map,pub_depth_cloud, pub_depth_points;
     
#if SINGLEDEPTH  
    message_filters::Subscriber<sensor_msgs::msgImage> sub_image;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_laserscan;
    message_filters::Synchronizer<syncPolicy> sync{syncPolicy(10)};
#else
    ros::Subscriber sub_image;
    ros::Subscriber sub_laserscan;
#endif

public:
    // data interface
    std::mutex *datamuex_ = nullptr;
    std::queue<vins::FrontEndResult::Ptr> *fontend_output_queue = nullptr;

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr syncCloud = nullptr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr imageCloud = nullptr;

    pcl::PointCloud<pcl::PointXYZI> normalize_point;
    pcl::KdTree<pcl::PointXYZI>::Ptr kdTree_ = nullptr;
    
    vector<vector<PointType>> pointsArray;

    FeatureTracker trackerData[NUM_OF_CAM];

    int pub_count = 1;

    bool first_image_flag = true;
    double first_image_time;
    double last_image_time = 0;

    bool init_pub = false;
    bool create_kd_tree=false;
    cv::Mat laservisual_alligned_img_;
    cv::Mat depth_img;
};


#endif //FRONTEND_H
