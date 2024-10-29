//
// Created by ubuntu on 2020/6/29.
//

#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

struct Imu {

public:
  typedef std::shared_ptr<Imu> Ptr;
  typedef std::shared_ptr<const Imu> ConstPtr;

public:
  Imu() {
    acc << 0., 0., 0.;
    gyr << 0., 0, 0;
    q_w_i.setIdentity();
  }
  Imu(double time, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
      : time(time), acc(acc), gyr(gyr) {}
  ~Imu() {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double time;
  Eigen::Vector3d acc; // accelerometer measurement (m^2/sec)
  Eigen::Vector3d gyr; // gyroscope measurement (rad/s)
  Eigen::Quaterniond q_w_i;
};

#endif // IMU_DATA_H
