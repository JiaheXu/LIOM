#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "Utils/EigenTypes.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>


#include <ceres/ceres.h>

#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/SE3AbsolutatePoseFactor.h"
#include "factor/SE3RelativtePoseFactor.h"
#include "Frontend/frontend_data.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


#define VISUAL_IMU_SUM_PRIOR
#define Enalbe_Lidar_Init 1
#define USE_ABSOLUTE_FACTOR 1

class Estimator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Estimator();

    void setParameter();

    // -----------------
    // imu preintegration frontend
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

    // visual and lidar frontend
    void
    processImageAndLidar(const vins::FrontEndResult &input);

    // receive relocation frontend
    void
    setReloFrame(double _frame_stamp,
                 int _frame_index,
                 Eigen::aligned_vector<Vector3d> &_match_points,
                 const Vector3d &_relo_t,
                 const Matrix3d &_relo_r);

    // -----------------

    // 梳理局部滑窗特征点的id
    void recomputeFrameId();
    // Estimator 重置
    inline void clearState();



    // -----------------
    // visual and imu initial
    bool initialStructure();
    bool visualInitialAlign();
    bool visualInitialAlignWithDepth();
    bool
    relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    // visual and imu initial
    // -----------------

    void slideWindow();
    void solveOdometry();
    inline void slideWindowNew();
    inline void slideWindowOld();
    void optimization();
    inline void vector2double();
    inline void double2vector();
    inline void updateFramePose();

    inline bool failureDetection();

private:

    inline int framedistance(vins::TimeFrameId frame0, vins::TimeFrameId frame1) const
    {
        CHECK(local_active_frames.find(frame0) != local_active_frames.end() and
            local_active_frames.find(frame1) != local_active_frames.end()
        ) << "frame0 or frame1 out of range";
        
        return time_frameid2_int_frameid.at(frame0) - time_frameid2_int_frameid.at(frame1);
    }
public:
    enum class SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum class MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;
    //extrinsic
    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    //VIO state vector
    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d last_R, last_R0;
    Vector3d last_P, last_P0;
    std_msgs::msg::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    Eigen::aligned_vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    Eigen::aligned_vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    Eigen::aligned_vector<Vector3d> point_cloud;
    Eigen::aligned_vector<Vector3d> margin_cloud;
    Eigen::aligned_vector<Vector3d> key_poses;
    double initial_timestamp;

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

#if 0
    // TODO: 原有的VINS-Mono 是将视觉和IMU 的先验和在一起
    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;
#else
    // TODO: 将视觉和IMU 的先验分开
    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    MarginalizationInfo *last_visual_marginalization_info;
    vector<double *> last_visual_marginalization_parameter_blocks;

    MarginalizationInfo *last_imu_marginalization_info;
    vector<double *> last_imu_marginalization_parameter_blocks;
#endif

    Eigen::aligned_map<double, ImageFrame> localWindowFrames; // LocalWindowFrames, 这个包含着不参与优化的关键帧
    std::set<double> local_active_frames;
    std::map<int, double> int_frameid2_time_frameid;
    std::map<double, int> time_frameid2_int_frameid;
    IntegrationBase *tmp_pre_integration;

    int failureCount;
    // henryzh47: VIO initialization status to be filled in odom cov
    int initial_stat; // 0: not ready; 1: ready
    // henryzh47: number of frames delay after initialization to avoid jerky pose
    //            before setting vio_initial_stat
    int init_delay_frame_countdown;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    Eigen::aligned_vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;
};
