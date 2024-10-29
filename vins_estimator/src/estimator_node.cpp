
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "estimator.h"
#include "Frontend/frontend.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "utility/MapRingBuffer.h"

// vio-loam frontend
namespace vins
{
typedef std::vector<std::pair<std::vector<sensor_msgs::msgImuConstPtr>, vins::FrontEndResult::Ptr>>
    imuAndFrontEndMeasurement;
} //namespace vins

Estimator estimator;

std::condition_variable con;

double current_time = -1;

queue<sensor_msgs::msgImuConstPtr> imu_buf;

queue<sensor_msgs::msgPointCloudConstPtr> feature_buf;

queue<sensor_msgs::msgPointCloudConstPtr> relo_buf;

queue<vins::FrontEndResult::Ptr> fontend_output_queue;

tbb::concurrent_bounded_queue<sensor_msgs::msgImuConstPtr> concurrent_imu_queue;

tbb::concurrent_bounded_queue<sensor_msgs::msgPointCloudConstPtr> concurrent_feature_queue;

tbb::concurrent_bounded_queue<vins::FrontEndResult::Ptr> concurrent_fontend_output_queue;

MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> laserimu_odom_buf;

int sum_of_wait = 0;

std::mutex m_buf;

std::mutex m_state;

std::mutex m_estimator;

double latest_time;

Eigen::Vector3d tmp_P;

Eigen::Quaterniond tmp_Q;

Eigen::Vector3d tmp_V;

Eigen::Vector3d tmp_Ba;

Eigen::Vector3d tmp_Bg;

Eigen::Vector3d acc_0;

Eigen::Vector3d gyr_0;

bool init_feature = 0;

bool init_imu = 1;

double last_imu_t = 0;

void predict(const sensor_msgs::msgImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu) {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

//    std_msgs::msg::Header header = imu_msg->header;
//    header.frame_id = "world";
//    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
//        pubLatestOdometry(estimator, tmp_P, tmp_Q, tmp_V, header);

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::msgImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msgImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<sensor_msgs::msgImuConstPtr>, sensor_msgs::msgPointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::msgImuConstPtr>, sensor_msgs::msgPointCloudConstPtr>> measurements;
    while (true) {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (imu_buf.back()->header.stamp.toSec() <= feature_buf.front()->header.stamp.toSec() + estimator.td) {
            //RCLCPP_WARN(this->get_logger(), "wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (imu_buf.front()->header.stamp.toSec() >= feature_buf.front()->header.stamp.toSec() + estimator.td) {
            RCLCPP_WARN(this->get_logger(), "throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msgPointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msgImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td) {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCLCPP_WARN(this->get_logger(), "no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

vins::imuAndFrontEndMeasurement
getImuAndFrontEndMeasurements()
{  
    bool laser_vio_sync=false;

    vins::imuAndFrontEndMeasurement measurements;
    while (true) {
        if (imu_buf.empty() || fontend_output_queue.empty())
            return measurements;

        if (imu_buf.back()->header.stamp.toSec() <= fontend_output_queue.front()->timestamp + estimator.td) {
            RCLCPP_WARN(this->get_logger(), "wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }


        if(!laserimu_odom_buf.empty()) {
            double lidar_odom_first_time, lidar_odom_last_time;
            laserimu_odom_buf.getFirstTime(lidar_odom_first_time);
            laserimu_odom_buf.getLastTime(lidar_odom_last_time);

            if (lidar_odom_last_time <=
                fontend_output_queue.front()->timestamp + estimator.td) {
                sum_of_wait++;
                laser_vio_sync=false;
             RCLCPP_WARN(this->get_logger(), "Lidar odometry is too old");
            }

            if (lidar_odom_first_time >
                fontend_output_queue.front()->timestamp + estimator.td) {
                laser_vio_sync=false;
                 RCLCPP_WARN(this->get_logger(), "Lidar odometry is too new");
                //TODO:可能会存在一些问题
                //fontend_output_queue.pop();
                //continue;
            }

            if(lidar_odom_first_time<fontend_output_queue.front()->timestamp+estimator.td and
              lidar_odom_last_time>fontend_output_queue.front()->timestamp+estimator.td)
            laser_vio_sync=true;
           

        }


        if (imu_buf.front()->header.stamp.toSec() >= fontend_output_queue.front()->timestamp + estimator.td) {
            RCLCPP_WARN(this->get_logger(), "throw img, only should happen at the beginning");
            fontend_output_queue.pop();
            continue;
        }
            
            auto frontend_msg = fontend_output_queue.front();

            if(laser_vio_sync==true) {
            double t_b_i = frontend_msg->timestamp + estimator.td;
            // 找到最近的
            auto after_ptr = laserimu_odom_buf.measMap_.upper_bound(t_b_i);
            auto before_ptr = after_ptr;
            before_ptr--;

            double ratio_bi =
                    (t_b_i - before_ptr->first) / (after_ptr->first - before_ptr->first);
        
            // LOG(INFO) << "[frontend_msg->Tw_imu_meas] delta t"
            //           << (t_b_i - before_ptr->first);
            // LOG(INFO) << "[frontend_msg->Tw_imu_meas] delta t"
            //           << (after_ptr->first - t_b_i);

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

            Eigen::Quaterniond q_w_original =
                    q_w_i_before.slerp(ratio_bi, q_w_i_after);

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


            Transformd T_imu_cam(estimator.ric[0], estimator.tic[0]);
            Transformd T_w_lidar(q_w_original, t_w_orignal);

            // TODO henryzh47: check this
            frontend_msg->Tw_imu_meas = T_w_lidar*Timu_lidar.inverse();
            
            Eigen::Vector3d v_w_i_before;
            v_w_i_before.x() = before_ptr->second->twist.twist.linear.x;
            v_w_i_before.y() = before_ptr->second->twist.twist.linear.y;
            v_w_i_before.z() = before_ptr->second->twist.twist.linear.z;
            
            Eigen::Vector3d v_w_i_after;
            v_w_i_after.x() = after_ptr->second->twist.twist.linear.x;
            v_w_i_after.y() = after_ptr->second->twist.twist.linear.y;
            v_w_i_after.z() = after_ptr->second->twist.twist.linear.z;

            // henryzh47: IMU velocity in IMU body frame (estimated from lidar odom)
            Eigen::Vector3d v_w_imu =
                    (1 - ratio_bi) * v_w_i_before + ratio_bi * v_w_i_after;

            // TODO henryzh47: doesn't seem to be necessary
            // V_w_lidar=q_w_original*V_w_lidar;

            // frontend_msg->vel_imu_meas=Timu_lidar.rot*V_w_lidar;  
            frontend_msg->vel_imu_meas=v_w_imu;  
            
            Eigen::Vector3d Bas;
            
            Bas.x()=after_ptr->second->pose.covariance[1];
            Bas.y()=after_ptr->second->pose.covariance[2];
            Bas.z()=after_ptr->second->pose.covariance[3];
            
            frontend_msg->Ba_meas=Bas;

            Eigen::Vector3d Bgs;
            Bgs.x()=after_ptr->second->pose.covariance[4];
            Bgs.y()=after_ptr->second->pose.covariance[5];
            Bgs.z()=after_ptr->second->pose.covariance[6];
            
            frontend_msg->Bg_meas=Bgs;
 
            frontend_msg->reset_id=(int)round(after_ptr->second->pose.covariance[0]);
            frontend_msg->gravity_meas=after_ptr->second->pose.covariance[7];
        
            frontend_msg->laser_odom_vio_sync=laser_vio_sync;
         //   LOG(INFO) << "[frontend_msg->Tw_imu_meas]" << frontend_msg->Tw_imu_meas<<"laser_odom_vio_sync:" << frontend_msg->laser_odom_vio_sync;

        }

        fontend_output_queue.pop();

        std::vector<sensor_msgs::msgImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < frontend_msg->timestamp + estimator.td) {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCLCPP_WARN(this->get_logger(), "no imu between two image");
        measurements.emplace_back(IMUs, frontend_msg);
    }

    return measurements;
}


// receive laser odomtry
void laser_imu_odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr laserOdometry) {
    m_buf.lock();
    laserimu_odom_buf.addMeas(laserOdometry, laserOdometry->header.stamp.toSec());
    m_buf.unlock();
}


void imu_callback(const sensor_msgs::msgImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t) {
        RCLCPP_WARN(this->get_logger(), "imu message in disorder! %f", imu_msg->header.stamp.toSec());
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

#if 1
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
#else
    concurrent_imu_queue.push(imu_msg);
#endif
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        //predict imu (no residual error)
        predict(imu_msg);
        std_msgs::msg::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(estimator,tmp_P, tmp_Q, tmp_V, header);
    }
}

void feature_callback(const sensor_msgs::msgPointCloudConstPtr &feature_msg)
{
    if (!init_feature) {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }

#if 1
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
#else
    concurrent_feature_queue.push(feature_msg);
#endif

    con.notify_one();
}

void restart_callback(const std_msgs::msg::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true) {
        RCLCPP_WARN(this->get_logger(), "restart the estimator!");
#if 1
        m_buf.lock();
        while (!feature_buf.empty())
            feature_buf.pop();
        while (!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
#else
        sensor_msgs::msgImuConstPtr imuMsgPtr;
        while (!concurrent_imu_queue.empty())
            concurrent_imu_queue.pop(imuMsgPtr);

        sensor_msgs::msgPointCloudConstPtr featurePtr;
        while (!concurrent_feature_queue.empty())
            concurrent_feature_queue.pop(featurePtr);
#endif
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::msgPointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
void process1()
{
    while (true) {
        vins::imuAndFrontEndMeasurement measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
        {
            return !(measurements = getImuAndFrontEndMeasurements()).empty();
        });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements) {
            auto &frontend_msg = measurement.second;
            auto &feature_msg = frontend_msg->feature;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first) {
                double t = imu_msg->header.stamp.toSec();
                double img_t = frontend_msg->timestamp + estimator.td;
                if (t <= img_t) {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            // set relocalization frame
            sensor_msgs::msgPointCloudConstPtr relo_msg = nullptr;
            while (!relo_buf.empty()) {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != nullptr) {
                Eigen::aligned_vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (const auto &point : relo_msg->points) {
                    Vector3d u_v_id;
                    u_v_id.x() = point.x;
                    u_v_id.y() = point.y;
                    u_v_id.z() = point.z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t
                    (relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3],
                                   relo_msg->channels[0].values[4],
                                   relo_msg->channels[0].values[5],
                                   relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            RCLCPP_DEBUG(this->get_logger(), "processing vision data with stamp %f \n", frontend_msg->timestamp);

            TicToc t_s;
            vins::FeatureTrackerResulst image;
            image = feature_msg;

            std_msgs::msg::Header header;
            header.stamp = rclcpp::Time(frontend_msg->timestamp*1e9);

            estimator.processImageAndLidar(*frontend_msg);

            double whole_t = t_s.toc();


            printStatistics(estimator, whole_t);

            header.frame_id = VINS_World_Frame;
            // utility/visualization.cpp

            pubOdometry(estimator, header);
            pubSlideWindowPoses(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            //pubWindowLidarPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != nullptr)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), rclcpp::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}
#pragma clang diagnostic pop

int main(int argc, char **argv)
{
    auto node = rclcpp::Node::make_shared("vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
    laserimu_odom_buf.allocate(5000);

    // initial frontend
    frontend opticalFlowEnd;
    opticalFlowEnd.setUpROS(nullptr, &n);

    opticalFlowEnd.datamuex_ = &m_buf;
    opticalFlowEnd.fontend_output_queue = &fontend_output_queue;
    // initial frontend end
#ifdef EIGEN_DONT_PARALLELIZE
    RCLCPP_DEBUG(this->get_logger(), "EIGEN_DONT_PARALLELIZE");
#endif
    RCLCPP_WARN(this->get_logger(), "waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
#if 0
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
#endif
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    //topic from pose_graph, notify if there's relocalization
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    ros::Subscriber sub_laserimu_odom = n.subscribe(ODOM_TOPIC, 2000, laser_imu_odom_callback);

    std::thread measurement_process{process1};

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
