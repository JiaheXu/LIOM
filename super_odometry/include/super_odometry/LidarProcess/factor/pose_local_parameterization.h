//
// Created by shiboz on 2021-02-06.
//

#ifndef SUPER_ODOMETRY_POSE_LOCAL_PARAMETERIZATION_H
#define SUPER_ODOMETRY_POSE_LOCAL_PARAMETERIZATION_H


#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../../utils/utility.h"
class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};






#endif //SUPER_ODOMETRY_POSE_LOCAL_PARAMETERIZATION_H