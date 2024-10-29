#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "Utils/EigenTypes.h"
#include "Frontend/frontend_data.h"


class KeypointObservation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeypointObservation() = default;
    KeypointObservation(const Eigen::Matrix<double, 8, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5);
        velocity.y() = _point(6);
        depth_measured = _point(7);
        cur_td = td;
    }

    Vector3d point;    // normalize point
    Vector2d uv;       // image point
    Vector2d velocity; // 相邻帧计算的特征点的速度
    double depth_measured;     // depth getted from lidar
    double cur_td;

    //bool is_used;      // TODO: 这个好像没什么用
};

class KeyPointLandmark
{
public:
    enum class EstimateFlag
    {
        NOT_INITIAL,
        DIRECT_MEASURED,
        TRIANGULATE
    };

    enum class SolveFlag
    {
        NOT_SOLVE,
        SOLVE_SUCC,
        SOLVE_FAIL
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyPointLandmark() = default;
    KeyPointLandmark(vins::FeatureID _feature_id, vins::TimeFrameId _start_frame, double _measured_depth)
        : feature_id(_feature_id),
          kf_id(_start_frame),
          start_frame(_start_frame), //TODO, start_frame 需要去掉
          used_num(0),
          estimated_depth(-1.0),
          measured_depth(_measured_depth),
          estimate_flag(EstimateFlag::NOT_INITIAL),
          solve_flag(SolveFlag::NOT_SOLVE)
    {
    }

public:

    vins::FeatureID feature_id; // landmark id
    vins::TimeFrameId kf_id;    // host frame id
    int start_frame;
    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    double measured_depth;     // TODO:这个好像没什么用

    EstimateFlag estimate_flag;//0 initial; 1 by depth image; 2 by triangulate
    //    EstimateFlag estimate_flag;
    SolveFlag solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
    Vector3d gt_p;
    // TODO: 使用这个代替
    Eigen::aligned_map<vins::TimeFrameId, KeypointObservation> obs;

    //　backend parameter interface
    std::array<double, SIZE_FEATURE> data;
};

class FeatureManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();

    bool addFeatureCheckParallax(int frame_count,
                                 const vins::TimeFrameId frame_id,
                                 const vins::FeatureTrackerResulst &image,
                                 double td);

    Eigen::aligned_vector<pair<Vector3d, Vector3d>>
    getCorresponding(int frame_count_l, int frame_count_r);

    Eigen::aligned_vector<pair<Vector3d, Vector3d>>
    getCorrespondingWithDepth(int frame_count_l, int frame_count_r);

    void
    triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);

    void
    triangulateWithDepth(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);

    // TODO 新加的函数,backend optimization interface
    //-------------------------
    void invDepth2Depth();
    void depth2InvDepth();
    void resetDepth();
    void removeFailures();
    //-------------------------


    // TODO: 重写某一帧观测的3d的函数
    //----------------------------

    /// \briefn 当滑窗开始优化时,边缘化一帧时, 3d点的主导帧,可能发生改变
    /// \param marg_frame_tid 要边缘化关键帧的id号
    /// \param Ps 滑窗中帧的position
    /// \param tic camera to imu transalation
    /// \param ric camera to imu roration
    void removeOneFrameObservationAndShiftDepth(vins::TimeFrameId marg_frame_tid,
                                                Vector3d Ps[],
                                                Vector3d tic[], Matrix3d ric[]
    );

    void
    removeOneFrameObservation(vins::TimeFrameId marg_frame_tid);

    //----------------------------
    // TODO: 重写某一帧观测的3d的函数



public:
    // estimator interface
    std::set<double> *local_active_frames_ = nullptr;
    std::map<int, double> *int_frameid2_time_frameid_ = nullptr;
    std::map<double, int> *time_frameid2_int_frameid_ = nullptr;

public:

    Eigen::aligned_unordered_map<vins::FeatureID, KeyPointLandmark> KeyPointLandmarks;

    int last_track_num;

private:
    double compensatedParallax2(const KeyPointLandmark &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif //FEATURE_MANAGER_H
