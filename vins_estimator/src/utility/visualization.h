#pragma once

#include <Eigen/Dense>
#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/bool.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/msg/marker.hpp>

#include "../estimator.h"
#include "../parameters.h"
#include "CameraPoseVisualization.h"

extern ros::Publisher pub_odometry, pub_loam_odometry, pub_latest_odometry;

extern ros::Publisher pub_path, pub_pose;

extern ros::Publisher pub_cloud, pub_map;

extern ros::Publisher pub_key_poses;

extern ros::Publisher pub_ref_pose, pub_cur_pose;

extern ros::Publisher pub_key;

extern nav_msgs::msg::Path path;

extern ros::Publisher pub_pose_graph;

extern int IMAGE_ROW, IMAGE_COL;

void registerPub(rclcpp::Node::SharedPtr node);

void pubLatestOdometry(const Estimator &estimator, const Eigen::Vector3d &P,
                       const Eigen::Quaterniond &Q, const Eigen::Vector3d &V,
                       const std_msgs::msg::Header &header);

void printStatistics(const Estimator &estimator, double t);

tf2::Transform transformConversion(const tf::StampedTransform& t);

void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubInitialGuess(const Estimator &estimator,
                     const std_msgs::msg::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubSlideWindowPoses(const Estimator &estimator,
                         const std_msgs::msg::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubWindowLidarPointCloud(const Estimator &estimator,
                              const std_msgs::msg::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::msg::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);
