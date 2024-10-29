//
// Created by ubuntu on 2020/5/26.
//


#include "frontend.h"
#include <gflags/gflags.h>
#include <glog/logging.h>


std::mutex mtx_lidar;

deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZI>::Ptr depth_cloud_unit_sphere(new pcl::PointCloud<pcl::PointXYZI>());


frontend::frontend()
{
    this->syncCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->imageCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    kdTree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    
    pointsArray.resize(num_bins);
    for (int i = 0; i < num_bins; ++i)
        pointsArray[i].resize(num_bins);

    // registe camera
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if (FISHEYE) {
        for (int i = 0; i < NUM_OF_CAM; i++) {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if (!trackerData[i].fisheye_mask.data) {
                LOG(INFO) << "load mask fail";
            }
            else
                LOG(INFO) << "load mask success";
        }
    }
} // function frontend end

void
frontend::setUpROS(ros::NodeHandle *pub_node, ros::NodeHandle *private_node)
{
    this->pub_node_ = pub_node;
    this->private_node_ = private_node;
    //ref: http://docs.ros.org/api/message_filters/html/c++/classmessage__filters_1_1TimeSynchronizer.html#a9e58750270e40a2314dd91632a9570a6
    //     https://blog.csdn.net/zyh821351004/article/details/47758433

#if SINGLEDEPTH
    sub_image.subscribe(*private_node_, IMAGE_TOPIC, 100);
    sub_laserscan.subscribe(*private_node_, LASER_TOPIC, 100);
    sync.connectInput(sub_image, sub_laserscan);
    sync.init();
    sync.registerCallback(boost::bind(&frontend::img_callback, this, _1, _2));
#else
    sub_image = private_node_->subscribe<sensor_msgs::msgImage>(IMAGE_TOPIC, 5, &frontend::img_callback, this);
    sub_laserscan = private_node->subscribe<sensor_msgs::msg::PointCloud2>(DEPTH_TOPIC, 5, &frontend::lidar_callback, this);
#endif
    // Registration Publisher
    pub_img = private_node_->advertise<sensor_msgs::msgPointCloud>("feature", 5);
    pub_match = private_node_->advertise<sensor_msgs::msgImage>("feature_img", 5);
    pub_depthimage = private_node_->advertise<sensor_msgs::msgImage>("depth_image", 5);
    pub_featuredepth = private_node_->advertise<sensor_msgs::msgImage>("feature/depth", 5);
    pub_restart = private_node_->advertise<std_msgs::msg::Bool>("restart", 5);

    pub_depth_cloud = private_node_->advertise<sensor_msgs::msg::PointCloud2>("visual/lidar_depth", 5);
    pub_lidar_map = private_node_->advertise<sensor_msgs::msg::PointCloud2>("visual/lidar_map", 5);
    pub_depth_points = private_node_->advertise<sensor_msgs::msg::PointCloud2>("visual/depth_points", 5);

} // function setUpROS

inline void
frontend::assign_feature_depth(cv::Point2f &p, double &depth)
{
    std::array<float, 3> pts = {p.x, p.y, 1};

    pcl::PointXYZI P;
    // 将视觉的特征点归一化到10这个平面上
    P.x = 10 * p.x;
    P.y = 10 * p.y;
    P.z = 10;

    // 根据当前视觉的特征点，对当前激光点搜索最近临，然后确定深度值
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqrDis;
    pointSearchInd.clear();
    pointSearchSqrDis.clear();
    kdTree_->nearestKSearch(P, 3, pointSearchInd, pointSearchSqrDis);

    double minDepth, maxDepth;
    // use lidar depth map to assign depth to feature
    // 如果搜索的距离小于0.5，并且能找到3个点
    if (pointSearchSqrDis[0] < 0.5 && pointSearchInd.size() == 3) {
        //取出该特征对应的激光点，恢复成真正的空间3D点
        pcl::PointXYZI depthPoint = normalize_point.points[pointSearchInd[0]];
        double x1 = depthPoint.x * depthPoint.intensity / 10;
        double y1 = depthPoint.y * depthPoint.intensity / 10;
        double z1 = depthPoint.intensity;
        minDepth = z1;
        maxDepth = z1;

        depthPoint = normalize_point.points[pointSearchInd[1]];
        double x2 = depthPoint.x * depthPoint.intensity / 10;
        double y2 = depthPoint.y * depthPoint.intensity / 10;
        double z2 = depthPoint.intensity;
        minDepth = (z2 < minDepth) ? z2 : minDepth;
        maxDepth = (z2 > maxDepth) ? z2 : maxDepth;

        depthPoint = normalize_point.points[pointSearchInd[2]];
        double x3 = depthPoint.x * depthPoint.intensity / 10;
        double y3 = depthPoint.y * depthPoint.intensity / 10;
        double z3 = depthPoint.intensity;
        minDepth = (z3 < minDepth) ? z3 : minDepth;
        maxDepth = (z3 > maxDepth) ? z3 : maxDepth;

        double u = pts[0];
        double v = pts[1];

        //根据三个激光点插值出深度值
        pts[2] = (x1 * y2 * z3 - x1 * y3 * z2 - x2 * y1 * z3 + x2 * y3 * z1 + x3 * y1 * z2 - x3 * y2 * z1)
            / (x1 * y2 - x2 * y1 - x1 * y3 + x3 * y1 + x2 * y3 - x3 * y2 + u * y1 * z2 - u * y2 * z1 - v * x1 * z2
                + v * x2 * z1 - u * y1 * z3 + u * y3 * z1 + v * x1 * z3 - v * x3 * z1 + u * y2 * z3 - u * y3 * z2
                - v * x2 * z3 + v * x3 * z2);
        // 如果插值出的深度是无限的
        if (!std::isfinite(pts[2])) {
            pts[2] = z1;
        }
        if (maxDepth - minDepth > 2) { // 如果最大距离-最小距离>2 ，则融合距离失败
            pts[2] = 0;
        }
        else if (pts[2] - maxDepth > 0.2) { //如果融合距离比最大的距离大0.2，
            pts[2] = maxDepth;
        }
        else if (pts[2] - minDepth < -0.2) {                      //如果融合距离比最小的距离还小
            pts[2] = minDepth; //那么选最小距离
        }
    }
    else // use lidar depth map to successfully assign depth feature end
    {
        pts[2] = 0;
    }// cannot find 3 lidar points to assign depth feature end

    depth = pts[2];
} // assign_feature_depth end

Eigen::Vector2f
frontend::xyz_to_uv(pcl::PointXYZ &xyz)
{
    float fx_ = fx;
    float fy_ = fy;
    float cx_ = cx;
    float cy_ = cy;
    //转换到图像平面点
    float x = fx_ * xyz.x + cx_ * xyz.z;
    float y = fy_ * xyz.y + cy_ * xyz.z;
    float z = xyz.z;

    //转换到图像uv
    Eigen::Vector2f uv(x / z, y / z);

    //如果没有distortion就直接返回uv
    if (!distortion_flag) {
        return uv;
    }
    else {
        // uv 去除图像畸变
        float xx = xyz.x / xyz.z;
        float yy = xyz.y / xyz.z;
        float r2 = xx * xx + yy * yy;
        float r4 = r2 * r2;
        float r6 = r4 * r2;
        float a1 = 2 * xx * yy;
        float a2 = r2 + 2 * xx * xx;
        float a3 = r2 + 2 * yy * yy;
        float cdist =
            1 + (float) distortion_img[0] * r2 + (float) distortion_img[1] * r4 + (float) distortion_img[4] * r6;
        float xd = xx * cdist + (float) distortion_img[2] * a1 + (float) distortion_img[3] * a2;
        float yd = yy * cdist + (float) distortion_img[2] * a3 + (float) distortion_img[3] * a1;
        Eigen::Vector2f uv_undist(xd * (float) fx_ + (float) cx_, yd * (float) fy_ + (float) cy_);

        return uv_undist;
    }
} // function xyz to_uv end


Eigen::Vector2f
frontend::xyz_to_uv(const Eigen::Vector3f &xyz)
{
    float fx_ = fx;
    float fy_ = fy;
    float cx_ = cx;
    float cy_ = cy;

    float x = fx_ * xyz(0) + cx_ * xyz(2);
    float y = fy_ * xyz(1) + cy_ * xyz(2);
    float z = xyz(2);
    Eigen::Vector2f uv(x / z, y / z);


    if (!distortion_flag) {
        return uv;
    }
    else {
        float xx = xyz(0) / xyz(2);
        float yy = xyz(1) / xyz(2);
        float r2 = xx * xx + yy * yy;
        float r4 = r2 * r2;
        float r6 = r4 * r2;
        float a1 = 2 * xx * yy;
        float a2 = r2 + 2 * xx * xx;
        float a3 = r2 + 2 * yy * yy;
        float cdist =
            1 + (float) distortion_img[0] * r2 + (float) distortion_img[1] * r4 + (float) distortion_img[4] * r6;
        float xd = xx * cdist + (float) distortion_img[2] * a1 + (float) distortion_img[3] * a2;
        float yd = yy * cdist + (float) distortion_img[2] * a3 + (float) distortion_img[3] * a1;
        Eigen::Vector2f uv_undist(xd * (float) fx_ + (float) cx_, yd * (float) fy_ + (float) cy_);

        return uv_undist;
    }
} // function xyz_to_uv end


bool
frontend::is_in_image(const Eigen::Vector2d &uv, int boundary, float scale)
{
    int u = static_cast<int> (uv(0) * scale);
    int v = static_cast<int> (uv(1) * scale);

    if (u > 0 + boundary && u < static_cast<int> (float(COL) * scale) - boundary && v > 0 + boundary
        && v < static_cast<int> (float(ROW) * scale) - boundary) {
        return true;
    }
    else {
        return false;
    }
} // function is_in_image end

void
frontend::show_image_with_points(cv::Mat &img, size_t num_level)
{
     if (pub_depthimage.get_subscription_count() != 0)
    {
     cv::Mat img_with_points = cv::Mat(cv::Size(img.cols, img.rows), CV_8UC3);

    if (img.type() == CV_32FC1) {
        cvtColor(img, img_with_points, cv::COLOR_GRAY2BGR);
    }
    else {
        //        img_with_points = img;
        img.copyTo(img_with_points);
    }

    //    cv::namedWindow("original_rgb",cv::WINDOW_NORMAL);
    //    cv::imshow("original_rgb",img_with_points);

    const float scale = 1.0f / (1 << num_level);

    // cerr << "Point size = " << imageCloud->size() << endl;
    int n = 0;
    for (auto &iter : *imageCloud) {
        n++;
        if (n % 5 != 0)
            continue;

        Eigen::Vector3d xyz_ref(iter.x, iter.y, iter.z);
        Eigen::Vector2d uv_ref;

        trackerData[0].m_camera->spaceToPlane(xyz_ref, uv_ref);

        const float u_ref_f = uv_ref(0);
        const float v_ref_f = uv_ref(1);
        const int u_ref_i = static_cast<int>(u_ref_f);
        const int v_ref_i = static_cast<int>(v_ref_f);



        //设置最大最远距离
        float v_min = 1.0;
        float v_max = 50.0;
        //设置距离差
        float dv = v_max - v_min;
        //取深度值
        float v = xyz_ref(2);
        //        if(v>30)
        //        cout<<"v: "<<v<<endl;
        float r = 1.0;
        float g = 1.0;
        float b = 1.0;
        if (v < v_min)
            v = v_min;
        if (v > v_max)
            v = v_max;

        if (v < v_min + 0.25 * dv) {
            r = 0.0;
            g = 4 * (v - v_min) / dv;
        }
        else if (v < (v_min + 0.5 * dv)) {
            r = 0.0;
            b = 1 + 4 * (v_min + 0.25 * dv - v) / dv;
        }
        else if (v < (v_min + 0.75 * dv)) {
            r = 4 * (v - v_min - 0.5 * dv) / dv;
            b = 0.0;
        }
        else {
            g = 1 + 4 * (v_min + 0.75 * dv - v) / dv;
            b = 0.0;
        }

        //       std::cout << "color: " << r << ", " << g << ", " << b << std::endl;
        //        iter->r = r;
        //        iter->g = g;
        //        iter->b = b;
        // TODO: 数据类型要一致
#if 1
        cv::circle(img_with_points,
                   cv::Point(u_ref_i, v_ref_i),
                   3.5,
                   cv::Scalar(static_cast<int> (r * 255), static_cast<int> (g * 255), static_cast<int> (b * 255)),
                   -1);
#else
        //　--------------- Error ---------------
        cv::circle(img_with_points, cv::Point(u_ref_i, v_ref_i), 3.5, cv::Scalar(r, g, b), -1);
        //　--------------- Error ---------------
#endif
    }

   
    cv_bridge::CvImage bridge;
    bridge.image = img_with_points;
    bridge.encoding = "rgb8";
    sensor_msgs::msgImage::Ptr imageShowPointer = bridge.toImageMsg();
    imageShowPointer->header.stamp= rclcpp::Time::now();
    pub_depthimage.publish(imageShowPointer);

  //  cv::imshow("image_with_points", img_with_points);
  //  cv::waitKey(1);
 }
}
float frontend::pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

void frontend::getColor(float p, float np, float &r, float &g, float &b)
{
    float inc = 6.0 / np;
    float x = p * inc;
    r = 0.0f;
    g = 0.0f;
    b = 0.0f;
    if ((0 <= x && x <= 1) || (5 <= x && x <= 6))
        r = 1.0f;
    else if (4 <= x && x <= 5)
        r = x - 4;
    else if (1 <= x && x <= 2)
        r = 1.0f - (x - 1);

    if (1 <= x && x <= 3)
        g = 1.0f;
    else if (0 <= x && x <= 1)
        g = x - 0;
    else if (3 <= x && x <= 4)
        g = 1.0f - (x - 3);

    if (3 <= x && x <= 5)
        b = 1.0f;
    else if (2 <= x && x <= 3)
        b = x - 2;
    else if (5 <= x && x <= 6)
        b = 1.0f - (x - 5);
    r *= 255.0;
    g *= 255.0;
    b *= 255.0;
}

float frontend::pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}
void
frontend::cloud_in_image(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudInput)
{
    int max_level_ = 2;
    float scale = 1.0f / (1 << max_level_);

    pcl::PointCloud<pcl::PointXYZRGBA> pc;
    normalize_point.clear();

    for (const auto &iter : *cloudInput) {
        //将点云的xyz,转换成uv
        pcl::PointXYZRGBA point;

        Eigen::Vector3d P(double(iter.x), double(iter.y), double(iter.z));
        Eigen::Vector2d uv;
        trackerData[0].m_camera->spaceToPlane(P, uv);

        //判断激光点云 是否在图像平面上
        if (is_in_image(uv, Boundary, scale)) { // && iter->z < 5.0

            int u = static_cast<int>(uv(0));
            int v = static_cast<int>(uv(1));

            //在KIIT图像上选取激光点的对应的像素坐标，并得到坐标对应的像素值
            //TODO: OpenCV 的数据类型不熟悉, 需要看一下OpenCV 图像访问的方法
            cv::Vec3b bgr = laservisual_alligned_img_.at<cv::Vec3b>(v, u);
            //将像素值附着给激光点云上，也就是让激光点云具备颜色信息
            point.x = iter.x;
            point.y = iter.y;
            point.z = iter.z;

            // TODO: 下面的操作要与OpenCV 的数据类型一致
#if 0
            point.r = static_cast<uint8_t>(bgr[2] * 255.0);
            point.g = static_cast<uint8_t>(bgr[1] * 255.0);
            point.b = static_cast<uint8_t>(bgr[0] * 255.0);
            point.a = 1.0;
#else
            point.r = static_cast<uint8_t>(bgr[2]);
            point.g = static_cast<uint8_t>(bgr[1]);
            point.b = static_cast<uint8_t>(bgr[0]);
            point.a = 1.0;
#endif
        }
        else {
            //如果激光点不合法，那么就采用绿色的点云
            point.r = static_cast<uint8_t>(0.0);
            point.g = static_cast<uint8_t>(255.0);
            point.b = static_cast<uint8_t>(0.0);
            point.a = 1;
        }
        //如果激光点大于0.0 就存储进来
        if (iter.z > 0.0) {
            pc.push_back(point);

            pcl::PointXYZI point;
            point.intensity = iter.z;
            point.x = iter.x * 10.f / iter.z;
            point.y = iter.y * 10.f / iter.z;
            point.z = 10.f;

            normalize_point.push_back(point);
        }
    }

    pcl::copyPointCloud(pc, *imageCloud);

//    LOG(INFO) << "imageCloud size: " << imageCloud->size();

    if(imageCloud->size()==0)
      return;

    kdTree_->setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(normalize_point)));
    
    show_image_with_points(laservisual_alligned_img_, 0);

    laservisual_alligned_img_.release();
} // cloud_in_image end

void frontend::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr laser_msg)
{
    static int lidar_count = -1;
    if (++lidar_count % (3 + 1) != 0)
        return;

    syncCloud.reset(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *syncCloud);

#if USE_LIDAT_DEPTH
    // 0. listen to transform
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    try
    {
        listener.waitForTransform(LIO_World_Frame, LIO_Laser_Frame, laser_msg->header.stamp, rclcpp::Duration(1.0));
        listener.lookupTransform(LIO_World_Frame,  LIO_Laser_Frame, laser_msg->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        RCLCPP_ERROR(this->get_logger(),"VIO frontend get lidar no tf from " << LIO_World_Frame << " to " << LIO_Laser_Frame << ": " << ex.what());
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform.getOrigin().x();
    yCur = transform.getOrigin().y();
    zCur = transform.getOrigin().z();
    tf::Matrix3x3 m(transform.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f T_w_lidar = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);
#else
    Eigen::Affine3f T_w_lidar = pcl::getTransformation(0, 0, 0, 0, 0, 0);
#endif


    if (LASER_TYPE == 64)
    {
        auto cloudSize = syncCloud->points.size();
        float verticalAngle;
        int rowIdn;
        pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
        pcl::PointXYZ thisPoint;
        for (size_t i = 0; i < cloudSize; ++i)
        {
            thisPoint.x = syncCloud->points[i].x;
            thisPoint.y = syncCloud->points[i].y;
            thisPoint.z = syncCloud->points[i].z;

            verticalAngle =
                atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % 4 == 0)
            {
                downsampled_cloud.push_back(thisPoint);
            }
        }

        *syncCloud = downsampled_cloud;
    }

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(syncCloud);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *syncCloud = *laser_cloud_in_ds;

    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)syncCloud->size(); ++i)
    {
        pcl::PointXYZ p = syncCloud->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            laser_cloud_in_filter->push_back(p);
    }
    *syncCloud = *laser_cloud_in_filter;

    // 4. offset T_lidar -> T_camera
#if 0  
    // pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());  
    // Eigen::Matrix<double, 4, 4> extrinsic_cam_laser;
    // extrinsic_cam_laser = Eigen::Matrix4d::Identity();
    // extrinsic_cam_laser.block<3, 3>(0, 0) = cam_laser_R;
    // extrinsic_cam_laser.block<3, 1>(0, 3) = cam_laser_T;
    // pcl::transformPointCloud(*syncCloud, *laser_cloud_offset, extrinsic_cam_laser);
    // *syncCloud = *laser_cloud_offset;
#else
    // 4. offset T_lidar -> T_camera
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    // TODO henryzh47: This transOffset is all 0?
    Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
    pcl::transformPointCloud(*syncCloud, *laser_cloud_offset, transOffset);
    *syncCloud = *laser_cloud_offset;
#endif



    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*syncCloud, *laser_cloud_global, T_w_lidar);

#if not fuse_global_point
    *depthCloud = *laser_cloud_global;
#else
    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        }
        else
        {
            break;
        }
    }

    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
        *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
#endif
    if(pub_lidar_map.get_subscription_count()!=0)
    publishCloud(&pub_lidar_map, depthCloud, laser_msg->header.stamp, VINS_World_Frame);
}




template <typename T>
sensor_msgs::msg::PointCloud2 frontend::publishCloud(ros::Publisher *thisPub, T thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}
bool frontend::create_sphere_cloud_kdtree(const rclcpp::Time &stamp_cur, cv::Mat cur_image, const pcl::PointCloud<PointType>::Ptr &depth_cloud_temp)
{

    // 0.3 look up transform at current image time
    static tf::TransformListener listener2;
    static tf::StampedTransform T_w_laser;
    try
    {
        // henryzh47: IMU_Frame is actually Lidar frame in LIO
        listener2.waitForTransform(LIO_World_Frame, LIO_Laser_Frame, stamp_cur, rclcpp::Duration(0.01));
        listener2.lookupTransform(LIO_World_Frame,  LIO_Laser_Frame, stamp_cur, T_w_laser);
    }
    catch (tf::TransformException ex)
    {
        RCLCPP_ERROR(this->get_logger(),"lidar no tf: "<<ex.what());
        return false;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = T_w_laser.getOrigin().x();
    yCur = T_w_laser.getOrigin().y();
    zCur = T_w_laser.getOrigin().z();
    tf::Matrix3x3 m(T_w_laser.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f T_w_l = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);
    Eigen::Affine3d T_w_l_ = T_w_l.cast<double>();
    Transformd T_w_laser_(T_w_l_.rotation(), T_w_l_.translation());
     
   
    // 0.2  check if depthCloud available
    if (depth_cloud_temp->size() == 0)
        return false;

    // 0.2 transform cloud from global frame to camera frame
    pcl::PointCloud<PointType>::Ptr depth_cloud_local(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*depth_cloud_temp, *depth_cloud_local, T_w_laser_.inverse().transform());

    // 0.3. project depth cloud on a range image, filter points satcked in the same region
    float bin_res = 180.0 / (float)num_bins; // currently only cover the space in front of lidar (-90 ~ 90)
    cv::Mat rangeImage = cv::Mat(num_bins, num_bins, CV_32F, cv::Scalar::all(FLT_MAX));
  


    for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
    {
        PointType p = depth_cloud_local->points[i];
        // TODO henryzh47: This assumes the rotation between camera and lidar
        // filter points not in camera view
        if (p.x < 0 || abs(p.y / p.x) > 10 || abs(p.z / p.x) > 10)
            continue;
        // find row id in range image
        float row_angle = atan2(p.z, sqrt(p.x * p.x + p.y * p.y)) * 180.0 / M_PI + 90.0; // degrees, bottom -> up, 0 -> 360
        int row_id = round(row_angle / bin_res);
        // find column id in range image
        float col_angle = atan2(p.x, p.y) * 180.0 / M_PI; // degrees, left -> right, 0 -> 360
        int col_id = round(col_angle / bin_res);
        // id may be out of boundary
        if (row_id < 0 || row_id >= num_bins || col_id < 0 || col_id >= num_bins)
            continue;
        // only keep points that's closer
        float dist = pointDistance(p);
        if (dist < rangeImage.at<float>(row_id, col_id))
        {
            rangeImage.at<float>(row_id, col_id) = dist;
            pointsArray[row_id][col_id] = p;
        }
    }
    
    // 0.4. extract downsampled depth cloud from range image
    pcl::PointCloud<PointType>::Ptr depth_cloud_local_filter2(new pcl::PointCloud<PointType>());
    for (int i = 0; i < num_bins; ++i)
    {
        for (int j = 0; j < num_bins; ++j)
        {
            if (rangeImage.at<float>(i, j) != FLT_MAX)
                depth_cloud_local_filter2->push_back(pointsArray[i][j]);
        }
    }

    *depth_cloud_local = *depth_cloud_local_filter2;

    if(pub_depth_cloud.get_subscription_count()!=0)
    publishCloud(&pub_depth_cloud, depth_cloud_local, stamp_cur, Laser_Frame);
    
    // 5. project depth cloud onto a unit sphere
    pcl::PointCloud<pcl::PointXYZI>::Ptr depth_cloud_unit_sphere_(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
    {
        pcl::PointXYZI point;
        PointType p;
        p = depth_cloud_local->points[i];
        float range = pointDistance(p);

        point.x = p.x / range;
        point.y = p.y / range;
        point.z = p.z / range;
        point.intensity = range;
        depth_cloud_unit_sphere_->push_back(point);
    }

    *depth_cloud_unit_sphere = *depth_cloud_unit_sphere_;
 
    if (depth_cloud_unit_sphere->size() < 10)
        return false;
    
     
    // 6. create a kd-tree using the spherical depth cloud
    kdTree_->setInputCloud(depth_cloud_unit_sphere);

  
    project_cloud_image(stamp_cur, cur_image, depth_cloud_local);
  
    return true;
}

void frontend::get_depth(const geometry_msgs::msg::Point32 &features_2d, pcl::PointXYZI &p)
{

   
    // 1. normalize 2d feature to a unit sphere
    Eigen::Vector3f feature_cur(features_2d.x, features_2d.y, features_2d.z); // z always equal to 1
    feature_cur.normalize();

    // convert to ROS standard
    p.x = feature_cur(2);
    p.y = -feature_cur(0);
    p.z = -feature_cur(1);
    p.intensity = -1;

    // 2. find the feature depth using kd-tree
    vector<int> pointSearchInd;
    vector<float> pointSearchSqDis;
    float bin_res = 180.0 / (float)num_bins; // currently only cover the space in front of lidar (-90 ~ 90)
    float dist_sq_threshold = pow(sin(bin_res / 180.0 * M_PI) * 5.0, 2);

    kdTree_->nearestKSearch(p, 3, pointSearchInd, pointSearchSqDis);


    if (pointSearchInd.size() == 3 && pointSearchSqDis[2] < dist_sq_threshold)
    {
        float r1 = depth_cloud_unit_sphere->points[pointSearchInd[0]].intensity;
        Eigen::Vector3f A(depth_cloud_unit_sphere->points[pointSearchInd[0]].x * r1,
                          depth_cloud_unit_sphere->points[pointSearchInd[0]].y * r1,
                          depth_cloud_unit_sphere->points[pointSearchInd[0]].z * r1);

        float r2 = depth_cloud_unit_sphere->points[pointSearchInd[1]].intensity;
        Eigen::Vector3f B(depth_cloud_unit_sphere->points[pointSearchInd[1]].x * r2,
                          depth_cloud_unit_sphere->points[pointSearchInd[1]].y * r2,
                          depth_cloud_unit_sphere->points[pointSearchInd[1]].z * r2);

        float r3 = depth_cloud_unit_sphere->points[pointSearchInd[2]].intensity;
        Eigen::Vector3f C(depth_cloud_unit_sphere->points[pointSearchInd[2]].x * r3,
                          depth_cloud_unit_sphere->points[pointSearchInd[2]].y * r3,
                          depth_cloud_unit_sphere->points[pointSearchInd[2]].z * r3);

        // https://math.stackexchange.com/questions/100439/determine-where-a-vector-will-intersect-a-plane
        Eigen::Vector3f V(p.x,
                          p.y,
                          p.z);

        Eigen::Vector3f N = (A - B).cross(B - C);
        float s = (N(0) * A(0) + N(1) * A(1) + N(2) * A(2)) / (N(0) * V(0) + N(1) * V(1) + N(2) * V(2));

        float min_depth = min(r1, min(r2, r3));
        float max_depth = max(r1, max(r2, r3));
        if (max_depth - min_depth > 2 || s <= 0.5)
        {  
          //  p.intensity=0;
            return;
        }
        else if (s - max_depth > 0)
        {
            s = max_depth;
        }
        else if (s - min_depth < 0)
        {
            s = min_depth;
        }
        // convert feature into cartesian space if depth is available
        p.x *= s;
        p.y *= s;
        p.z *= s;
        // the obtained depth here is for unit sphere, VINS estimator needs depth for normalized feature (by value z), (lidar x = camera z)
        p.intensity = p.x;
    }
}

void frontend::project_cloud_image(const rclcpp::Time &stamp_cur, cv::Mat imageCur, const pcl::PointCloud<PointType>::Ptr &depth_cloud_local)
{
    // visualization project points on image for visualization (it's slow!)
    if (pub_depthimage.get_subscription_count() != 0)
    {  
       
        vector<cv::Point2f> points_2d;
        vector<float> points_distance;

        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            // convert points from 3D to 2D
            Eigen::Vector3d p_3d(-depth_cloud_local->points[i].y,
                                 -depth_cloud_local->points[i].z,
                                 depth_cloud_local->points[i].x);
            Eigen::Vector2d p_2d;
            trackerData[0].m_camera->spaceToPlane(p_3d, p_2d);
            points_2d.push_back(cv::Point2f(p_2d(0), p_2d(1)));
            points_distance.push_back(pointDistance(depth_cloud_local->points[i]));
        }

        cv::Mat showImage, circleImage;
        showImage = imageCur.clone();
        //cv::cvtColor(imageCur, showImage, cv::COLOR_GRAY2RGB);
        circleImage = showImage.clone();

        for (int i = 0; i < (int)points_2d.size(); ++i)
        {
            float r, g, b;
            getColor(points_distance[i], 50.0, r, g, b);
            cv::circle(circleImage, points_2d[i], 0, cv::Scalar(r, g, b), 5);
        }

        cv::addWeighted(showImage, 1.0, circleImage, 0.7, 0, showImage); // blend camera image and circle image

        cv_bridge::CvImage bridge;
        bridge.image = showImage;
        bridge.encoding = "rgb8";
        sensor_msgs::msgImage::Ptr imageShowPointer = bridge.toImageMsg();
        imageShowPointer->header.stamp = stamp_cur;
      
        pub_depthimage.publish(imageShowPointer);
    }
}

#if SINGLEDEPTH 
void frontend::img_callback(const sensor_msgs::msgImageConstPtr &color_msg,
                            const sensor_msgs::msg::PointCloud2::SharedPtr lasercloud_msg)
{
  
    //cerr << "color_msg: " << color_msg->encoding << endl;
    if (first_image_flag) {
        first_image_flag = false;
        first_image_time = color_msg->header.stamp.toSec();
        last_image_time = color_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (color_msg->header.stamp.toSec() - last_image_time > 1.0 || color_msg->header.stamp.toSec() < last_image_time) {
        RCLCPP_WARN(this->get_logger(), "image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = color_msg->header.stamp.toSec();
    // frequency control
    if (round(1.0 * pub_count / (color_msg->header.stamp.toSec() - first_image_time)) <= FREQ) {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (color_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ) {
            first_image_time = color_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;
    // encodings in ros: http://docs.ros.org/diamondback/api/sensor_msgs/html/image__encodings_8cpp_source.html
    // color has encoding RGB8
    cv_bridge::CvImageConstPtr ptr;
    if (color_msg->encoding == "8UC1") {
        //shan:why 8UC1 need this operation? Find answer:https://github.com/ros-perception/vision_opencv/issues/175
        sensor_msgs::msgImage img;
        img.header = color_msg->header;
        img.height = color_msg->height;
        img.width = color_msg->width;
        img.is_bigendian = color_msg->is_bigendian;
        img.step = color_msg->step;
        img.data = color_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::msgimage_encodings::MONO8);
        RCLCPP_INFO(this->get_logger(),"MONO_FORMAT!");
    }
    else {
        ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::msgimage_encodings::MONO8);
    }
    cv::Mat rgb;
#if 0
    // TODO: Error OpenCV Mat 赋值操作是指针操作, 你执行cvtColor(ptr->image, rgb,)将会修改ptr->image, 这不是你要的表达意思
    rgb = ptr->image;
#else

#endif
    
    cvtColor(ptr->image, rgb, cv::COLOR_GRAY2BGR);

    if (rgb.type() != CV_8UC3) {
        RCLCPP_ERROR(this->get_logger(),"input image type != CV_8UC3");
    }
    laservisual_alligned_img_ = rgb;
    
    //TODO: add the downsample images
    // cv::resized_image;
    // int down_width = 640;
    // int down_height = 480;
    // resize(laservisual_alligned_img_, resized_image, Size(down_width, down_height), INTER_LINEAR);



    // 同步的点云

    syncCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*lasercloud_msg, *syncCloud);
    if (LASER_TYPE == 64) {
        auto cloudSize = syncCloud->points.size();
        float verticalAngle;
        int rowIdn;
        pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
        pcl::PointXYZ thisPoint;
        for (size_t i = 0; i < cloudSize; ++i) {
            thisPoint.x = syncCloud->points[i].x;
            thisPoint.y = syncCloud->points[i].y;
            thisPoint.z = syncCloud->points[i].z;

            verticalAngle =
                atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % 4 == 0) {
                downsampled_cloud.push_back(thisPoint);
            }
        }

        *syncCloud = downsampled_cloud;
    }


    Eigen::Matrix<double, 4, 4> extrinsic_cam_laser;

    extrinsic_cam_laser = Eigen::Matrix4d::Identity();
    extrinsic_cam_laser.block<3, 3>(0, 0) = cam_laser_R;
    extrinsic_cam_laser.block<3, 1>(0, 3) = cam_laser_T;


    //   std::cout<<"matrix:"<<extrinsic_cam_laser<<std::endl;
    // 将激光数据转化到相机坐标系下

    pcl::transformPointCloud(*syncCloud, *syncCloud, extrinsic_cam_laser);

    cloud_in_image(syncCloud);

    //
    //    //depth has encoding TYPE_16UC1
    //    cv_bridge::CvImageConstPtr depth_ptr;
    //    // debug use     std::cout<<depth_msg->encoding<<std::endl;
    //    {
    //        sensor_msgs::msgImage img;
    //        img.header = depth_msg->header;
    //        img.height = depth_msg->height;
    //        img.width = depth_msg->width;
    //        img.is_bigendian = depth_msg->is_bigendian;
    //        img.step = depth_msg->step;
    //        img.data = depth_msg->data;
    //        img.encoding = sensor_msgs::msgimage_encodings::MONO16;
    //        depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::msgimage_encodings::MONO16);
    //    }

# if 1
    cv::Mat show_img = ptr->image;
    TicToc t_r;
    // init pts here, using readImage()

    for (int i = 0; i < NUM_OF_CAM; i++) {
        RCLCPP_DEBUG(this->get_logger(), "processing camera %d", i);
        if (i != 1 || !STEREO_TRACK) {
            //输入深度值
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)),
                                     ptr->image.rowRange(ROW * i, ROW * (i + 1)),
                                     color_msg->header.stamp.toSec());
        }
        else {
            if (EQUALIZE) {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else {
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }

        }
        //always 0
#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }
    // update all id in ids[]
    //If has ids[i] == -1 (newly added pts by cv::goodFeaturesToTrack), substitute by gloabl id counter (n_id)
    for (unsigned int i = 0;; i++) {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }
    if (PUB_THIS_FRAME) {

        pub_count++;
        //http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
        sensor_msgs::msgPointCloudPtr feature_points(new sensor_msgs::msgPointCloud);
        sensor_msgs::msgChannelFloat32 id_of_point;
        sensor_msgs::msgChannelFloat32 u_of_point;
        sensor_msgs::msgChannelFloat32 v_of_point;
        sensor_msgs::msgChannelFloat32 velocity_x_of_point;
        sensor_msgs::msgChannelFloat32 velocity_y_of_point;
        //Use round to get depth value of corresponding points
        sensor_msgs::msgChannelFloat32 depth_of_point;

        feature_points->header = color_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        if (SHOW_TRACK) {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::msgimage_encodings::BGR8);
            depth_img = ptr->image.clone();
        }

        /// Publish FeatureTrack Result
        // 视觉追踪的结果
        // ROS_INFO_STREAM("numwithdepth: " << numwithdepth);
        // Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
        // xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
        // image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
        vins::FrontEndResult::Ptr output = std::make_shared<vins::FrontEndResult>();

        output->timestamp = color_msg->header.stamp.toSec();
        // step1: publish pointcloud
        output->cloud_ptr = syncCloud;
        output->image = depth_img.clone();

        // step2: publish featureTrack result
        int numwithdepth = 0;
        for (int i = 0; i < NUM_OF_CAM; i++) {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (size_t j = 0; j < ids.size(); j++) {
                if (trackerData[i].track_cnt[j] > 1) {
                    int p_id = ids[j];
                    //not used
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;
                    // push normalized point to pointcloud
                    feature_points->points.push_back(p);
                    // push other info
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);

                    //nearest neighbor....fastest  may be changed
                    cv::Point2f pts(p.x, p.y);
                    double depth;
                    //TODO: Calculate point depth
                    //TODO: 还是不认真, 需要培养耐性
                    depth=-1;
                    assign_feature_depth(pts, depth);
                    depth_of_point.values.push_back(depth);
                    // ROS_INFO_STREAM("depth: "<< depth);


                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                    xyz_uv_velocity_depth << p.x, p.y, p.z, cur_pts[j].x, cur_pts[j].y, pts_velocity[j]
                        .x, pts_velocity[j].y, depth;
                    // image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
                    output->feature[p_id].emplace_back(i, xyz_uv_velocity_depth);


                    if (depth > 0)
                        numwithdepth++;
                    if (SHOW_TRACK) {
                        if (depth > 0) {
                            // 蓝色
                            cv::circle(depth_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255, 0, 0), 2);
                        }
                        else {
                            // 绿色
                            cv::circle(depth_img, trackerData[i].cur_pts[j], 2, cv::Scalar(0, 255, 0), 2);
                        }
                    }
                }
            }
        }

        datamuex_->lock();
        fontend_output_queue->push(output);
        datamuex_->unlock();

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        feature_points->channels.push_back(depth_of_point);
        RCLCPP_DEBUG(this->get_logger(), "publish %f, at %f", feature_points->header.stamp.toSec(), rclcpp::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub) {
            init_pub = 1;
        }
        else {
            pub_img.publish(feature_points);//"feature"
        }
        // Show image with tracked points in rviz (by topic pub_match)
        if (SHOW_TRACK) {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::msgimage_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++) {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);

                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);//??seems useless?

                for (size_t j = 0; j < trackerData[i].cur_pts.size(); j++) {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());
            sensor_msgs::msgImagePtr depthImgMsg = cv_bridge::CvImage(ptr->header, "bgr8", depth_img).toImageMsg();
            pub_featuredepth.publish(depthImgMsg);
        }
    }


    //RCLCPP_INFO(this->get_logger(),"whole feature tracker processing costs: %f", t_r.toc());
#endif

}
# else
void frontend::img_callback(const sensor_msgs::msgImageConstPtr &color_msg)
{
    //step1: first image, frequence control and format conversion
    if (first_image_flag)
    {
        first_image_flag = false;
        first_image_time = color_msg->header.stamp.toSec();
        last_image_time = color_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (color_msg->header.stamp.toSec() - last_image_time > 1.0 || color_msg->header.stamp.toSec() < last_image_time)
    {
        RCLCPP_WARN(this->get_logger(), "image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }

    last_image_time = color_msg->header.stamp.toSec();
    // frequency control
    if (round(1.0 * pub_count / (color_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (color_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = color_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;
    // encodings in ros: http://docs.ros.org/diamondback/api/sensor_msgs/html/image__encodings_8cpp_source.html
    // color has encoding RGB8
    cv_bridge::CvImageConstPtr ptr;
    if (color_msg->encoding == "8UC1")
    {
        //shan:why 8UC1 need this operation? Find answer:https://github.com/ros-perception/vision_opencv/issues/175
        sensor_msgs::msgImage img;
        img.header = color_msg->header;
        img.height = color_msg->height;
        img.width = color_msg->width;
        img.is_bigendian = color_msg->is_bigendian;
        img.step = color_msg->step;
        img.data = color_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::msgimage_encodings::MONO8);
        RCLCPP_INFO(this->get_logger(),"MONO_FORMAT!");
    }
    else
    {
        ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::msgimage_encodings::MONO8);
    }
    cv::Mat rgb;
#if 0
    // TODO: Error OpenCV Mat 赋值操作是指针操作, 你执行cvtColor(ptr->image, rgb,)将会修改ptr->image, 这不是你要的表达意思
    rgb = ptr->image;
#endif

    cvtColor(ptr->image, rgb, cv::COLOR_GRAY2BGR);

    if (rgb.type() != CV_8UC3)
    {
        RCLCPP_ERROR(this->get_logger(),"input image type != CV_8UC3");
    }

    laservisual_alligned_img_ = rgb;

// step2: process image and achieve feature detection

    cv::Mat show_img = ptr->image;
    TicToc t_r;

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCLCPP_DEBUG(this->get_logger(), "processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)
        {
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)),
                                     ptr->image.rowRange(ROW * i, ROW * (i + 1)),
                                     color_msg->header.stamp.toSec());
        }
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
            {
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }
        }
        //always 0
#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    // update all id in ids[]
    //If has ids[i] == -1 (newly added pts by cv::goodFeaturesToTrack), substitute by gloabl id counter (n_id)
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }


    // step3: assign depth for visual features
    if (PUB_THIS_FRAME)
    {

        pub_count++;
        //http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
        sensor_msgs::msgPointCloudPtr feature_points(new sensor_msgs::msgPointCloud);
        sensor_msgs::msgChannelFloat32 id_of_point;
        sensor_msgs::msgChannelFloat32 u_of_point;
        sensor_msgs::msgChannelFloat32 v_of_point;
        sensor_msgs::msgChannelFloat32 velocity_x_of_point;
        sensor_msgs::msgChannelFloat32 velocity_y_of_point;
        //Use round to get depth value of corresponding points
        sensor_msgs::msgChannelFloat32 depth_of_point;

        feature_points->header = color_msg->header;
        feature_points->header.frame_id = Camera_Frame;

        vector<set<int>> hash_ids(NUM_OF_CAM);

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::msgimage_encodings::BGR8);
            depth_img = ptr->image.clone();
        }

        /// Publish FeatureTrack Result
        // 视觉追踪的结果
        // ROS_INFO_STREAM("numwithdepth: " << numwithdepth);
        // Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
        // xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
        // image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);

        vins::FrontEndResult::Ptr output = std::make_shared<vins::FrontEndResult>();
        output->timestamp = color_msg->header.stamp.toSec();
        output->cloud_ptr = syncCloud;
        output->image = depth_img.clone();

        //3.1 create the kd tree for local point cloud
        pcl::PointCloud<PointType>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType>());
        mtx_lidar.lock();
        *depth_cloud_temp = *depthCloud;
        mtx_lidar.unlock();
    
        create_kd_tree = create_sphere_cloud_kdtree(color_msg->header.stamp, output->image, depth_cloud_temp);
       
        //3.2 publish featureTrack result
        int numwithdepth = 0;

        pcl::PointCloud<pcl::PointXYZI>::Ptr features_3d_sphere(new pcl::PointCloud<pcl::PointXYZI>());
         
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (size_t j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    //not used
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;
                    // push normalized point to pointcloud
                    feature_points->points.push_back(p);
                    // push other info
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);

                    pcl::PointXYZI featuredepth;
                    // 3.3 assign depth for current visual features
                    if (create_kd_tree)
                         get_depth(p, featuredepth);

                    features_3d_sphere->push_back(featuredepth);
                    depth_of_point.values.push_back(featuredepth.intensity);

                    Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                    xyz_uv_velocity_depth << p.x, p.y, p.z, cur_pts[j].x, cur_pts[j].y, pts_velocity[j].x, pts_velocity[j].y, featuredepth.intensity;
                    // image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
                    output->feature[p_id].emplace_back(i, xyz_uv_velocity_depth);

                    
                    if (featuredepth.intensity > 0)
                        numwithdepth++;
                    if (SHOW_TRACK) {
                        if (featuredepth.intensity > 0) {
                       
                            cv::circle(depth_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255, 0, 0), 2);
                        }
                        else {
                          
                            cv::circle(depth_img, trackerData[i].cur_pts[j], 2, cv::Scalar(0, 255, 0), 2);
                        }



                }
            }
        }

        datamuex_->lock();
        fontend_output_queue->push(output);
        datamuex_->unlock();

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        feature_points->channels.push_back(depth_of_point);
        RCLCPP_DEBUG(this->get_logger(), "publish %f, at %f", feature_points->header.stamp.toSec(), rclcpp::Time::now().toSec());

        // visualize features in cartesian 3d space (including the feature without depth (default 1))
        if(pub_depth_points.get_subscription_count()!=0)
        publishCloud(&pub_depth_points, features_3d_sphere, feature_points->header.stamp, Laser_Frame);
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
        {   
            if(pub_img.get_subscription_count()!=0)
            pub_img.publish(feature_points); //"feature"
        }
        // step 4. Show image with tracked points in rviz (by topic pub_match)
        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::msgimage_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);

                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB); //??seems useless?

                for (size_t j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            if(pub_match.get_subscription_count()!=0)
            pub_match.publish(ptr->toImageMsg());
            sensor_msgs::msgImagePtr depthImgMsg = cv_bridge::CvImage(ptr->header, "bgr8", depth_img).toImageMsg();
            if(pub_featuredepth.get_subscription_count()!=0)
            pub_featuredepth.publish(depthImgMsg);
        }
    }

    RCLCPP_WARN(this->get_logger(), "num of feature with depth: %d", numwithdepth);
    //RCLCPP_INFO(this->get_logger(),"whole feature tracker processing costs: %f", t_r.toc());
  }
}
#endif
