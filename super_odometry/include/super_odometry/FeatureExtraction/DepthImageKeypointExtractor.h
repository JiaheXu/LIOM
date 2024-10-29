//
// Created by shiboz on 2021-09-04.
//

#ifndef SUPER_ODOMETRY_DEPTHIMAGEKEYPOINTEXTRACTOR_H
#define SUPER_ODOMETRY_DEPTHIMAGEKEYPOINTEXTRACTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "rclcpp/rclcpp.hpp"
//points covariance class
class Double2d{
public:
    int id;
    double value;
    Double2d(int id_in, double value_in);
};


class DepthKeypointExtractor
{
public:
    DepthKeypointExtractor();
    void uniformfeatureExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf,int skip_num);
    void featureExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out_surf);
    void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out_surf);

};


#endif //SUPER_ODOMETRY_DEPTHIMAGEKEYPOINTEXTRACTOR_H
