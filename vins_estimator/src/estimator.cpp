#include "estimator.h"
#include "Utils/EigenTypes.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

Estimator::Estimator()
    : f_manager{Rs}
{   
    failureCount = -1;
    LOG(INFO) << "estimator init begins";
    // 向 feature_manager 传递frame 管理器的指针
    f_manager.int_frameid2_time_frameid_ = &int_frameid2_time_frameid;
    f_manager.time_frameid2_int_frameid_ = &time_frameid2_int_frameid;
    f_manager.local_active_frames_ = &local_active_frames;

    clearState();
}

void
Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

void
Estimator::clearState()
{  
    ++failureCount;

    local_active_frames.clear();
    int_frameid2_time_frameid.clear();
    time_frameid2_int_frameid.clear();

    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }
    for (auto &it : localWindowFrames) {
        if (it.second.pre_integration != nullptr) {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }
    solver_flag = SolverFlag::INITIAL;
    first_imu = false,
        sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;

    initial_timestamp = 0;
    localWindowFrames.clear();
    td = TD;


    if (tmp_pre_integration)
        delete tmp_pre_integration;
#ifdef VISUAL_IMU_SUM_PRIOR
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;
#else
    if (last_visual_marginalization_info)
        delete last_visual_marginalization_info;
    if (last_imu_marginalization_info)
        delete last_imu_marginalization_info;
#endif

    tmp_pre_integration = nullptr;

#ifdef VISUAL_IMU_SUM_PRIOR
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();
#else
    last_visual_marginalization_info = nullptr;
    last_visual_marginalization_parameter_blocks.clear();

    last_imu_marginalization_info = nullptr;
    last_imu_marginalization_parameter_blocks.clear();
#endif

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();

     // henryzh47: VIO initialization stat for LIO
    initial_stat = 0;
    init_delay_frame_countdown = VIO_INIT_DELAY;
}

void
Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu) {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count]) {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0) {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// featureID-> [<camID, xyz, uv, velocity, depth>]
void
Estimator::processImageAndLidar(const vins::FrontEndResult &input)
{
    RCLCPP_DEBUG(this->get_logger(), "new image coming ------------------------------------------");
    RCLCPP_DEBUG(this->get_logger(), "Adding feature points %lu", input.feature.size());
    // FeaturePerFrame
    // FeaturePerId
    // feature
    local_active_frames.insert(input.timestamp);
    recomputeFrameId(); // 梳理局部滑窗中frame的id

 //   LOG(INFO) << "local_active_frames.size: " << local_active_frames.size();
    //    if (f_manager.addFeatureCheckParallax(frame_count, input.feature, td))
    if (f_manager.addFeatureCheckParallax(frame_count, input.timestamp, input.feature, td))
        marginalization_flag = MarginalizationFlag::MARGIN_OLD;
    else
        marginalization_flag = MarginalizationFlag::MARGIN_SECOND_NEW;

    RCLCPP_DEBUG(this->get_logger(), "this frame is--------------------%s",
              marginalization_flag == MarginalizationFlag::MARGIN_OLD ? "reject" : "accept");
    RCLCPP_DEBUG(this->get_logger(), "%s", marginalization_flag == MarginalizationFlag::MARGIN_OLD ? "Non-keyframe" : "Keyframe");
    RCLCPP_DEBUG(this->get_logger(), "Solving %d", frame_count);
    RCLCPP_DEBUG(this->get_logger(), "number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count].stamp = rclcpp::Time(input.timestamp*1e9);

    //    {
    //
    //        int count = 0;
    //        std::string output;
    //        for (const auto tid : Headers) {
    //            output += std::to_string(count) + " ->" + std::to_string(tid.stamp.toSec()) + "\n";
    //            count++;
    //        }
    //
    //        LOG(INFO) << "Original WINDOW Frame: \n" << output;
    //    }
    //step 创建新的一帧数据
    ImageFrame imageframe(input);
    imageframe.pre_integration = tmp_pre_integration;
    localWindowFrames.insert(make_pair(input.timestamp, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if (ESTIMATE_EXTRINSIC == 2) {
        RCLCPP_INFO(this->get_logger(),"calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0) {
            Eigen::aligned_vector<pair<Vector3d, Vector3d>>
                corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric)) {
                RCLCPP_WARN(this->get_logger(), "initial extrinsic rotation calib success");
                RCLCPP_WARN_STREAM(this->get_logger(), "initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    if (solver_flag == SolverFlag::INITIAL) {
        if (frame_count == WINDOW_SIZE) {
            bool result = false;
            if (ESTIMATE_EXTRINSIC != 2 && (input.timestamp - initial_timestamp) > 0.1) {
                result = initialStructure();
                initial_timestamp = input.timestamp;
            }
            //if init sfm success
            if (result) {
                solver_flag = SolverFlag::NON_LINEAR;
                solveOdometry();
                slideWindow();
                f_manager.removeFailures();
                LOG(INFO) << "Initialization finish!";
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];

            }
            else
                slideWindow();
        }
        else
            frame_count++;
    }
    else {
        TicToc t_solve;
        solveOdometry();
        LOG(ERROR) << "solver costs: " << t_solve.toc() << "ms";

        if (failureDetection()) {
            LOG(ERROR) << "failure detection!";
            failure_occur = true;
            clearState();
            setParameter();
            LOG(ERROR) << "system reboot!";
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();
//        LOG(INFO) << "marginalization costs: " << t_margin.toc() << "ms";
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }

     // henryzh47: update VIO initial stat
    if (init_delay_frame_countdown-- >= 0)
    {
            RCLCPP_WARN(this->get_logger(), "VIO initialization buffer frames");
    }
    else
    {
            initial_stat = 1;
    }

    recomputeFrameId();
}// function processImageAndLidar


void Estimator::recomputeFrameId()
{

    int_frameid2_time_frameid.clear();
    time_frameid2_int_frameid.clear();

    int localwindow_id = 0;
    std::string output;
    for (const auto tid : local_active_frames) {
        int_frameid2_time_frameid[localwindow_id] = tid;
        time_frameid2_int_frameid[tid] = localwindow_id;
        output += std::to_string(localwindow_id) + " ->" + std::to_string(tid) + "\n";

        localwindow_id++;
    }
    //LOG(INFO) << "WINDOW Frame: \n" << output;

}// recomputeFrameId

bool
Estimator::initialStructure()
{   
   
#if Enalbe_Lidar_Init
    // Lidar initialization
    {
        // TODO henryzh47: set to false for debug
        bool lidar_info_available = true;

        // clear key frame in the container
        for (map<double, ImageFrame>::iterator frame_it = localWindowFrames.begin(); frame_it != localWindowFrames.end(); frame_it++)
            frame_it->second.is_key_frame = false;

        // check if lidar info in the window is valid
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            if (localWindowFrames[Headers[i].stamp.toSec()].reset_id < 0 ||
                localWindowFrames[Headers[i].stamp.toSec()].reset_id != localWindowFrames[Headers[0].stamp.toSec()].reset_id || localWindowFrames[Headers[i].stamp.toSec()].laser_odom_vio_sync == false)
            {
                // lidar odometry not available (id=-1) or lidar odometry relocated due to pose correction
                lidar_info_available = false;
                RCLCPP_INFO(this->get_logger(),"Lidar initialization info not enough.");
                break;
            }
        }

        if (lidar_info_available == true)
        {
            // Update state
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                Vector3d P_meas(localWindowFrames[Headers[i].stamp.toSec()].Tw_imu_meas.pos.x(),
                                localWindowFrames[Headers[i].stamp.toSec()].Tw_imu_meas.pos.y(),
                                localWindowFrames[Headers[i].stamp.toSec()].Tw_imu_meas.pos.z());

                Ps[i] = P_meas;
                Rs[i] = localWindowFrames[Headers[i].stamp.toSec()].Tw_imu_meas.rotationMatrix();
                Vs[i] = localWindowFrames[Headers[i].stamp.toSec()].vel_imu_meas;
                Bas[i] = localWindowFrames[Headers[i].stamp.toSec()].Ba_meas;
                Bgs[i] = localWindowFrames[Headers[i].stamp.toSec()].Bg_meas;

                pre_integrations[i]->repropagate(Bas[i], Bgs[i]);

                localWindowFrames[Headers[i].stamp.toSec()].is_key_frame = true;
            }

            Matrix3d relative_R;
            Vector3d relative_T;
            int l;
            if (!relativePose(relative_R, relative_T, l))
            {
                RCLCPP_WARN(this->get_logger(), "Not enough parallex when initializing from lidar info");
                return false;
            }

            // update gravity
            g = Eigen::Vector3d(0, 0, localWindowFrames[Headers[0].stamp.toSec()].gravity_meas);

            // reset all features
            f_manager.resetDepth();

            // triangulate all features
            Vector3d TIC_TMP[NUM_OF_CAM];
            for (int i = 0; i < NUM_OF_CAM; i++)
                TIC_TMP[i].setZero();
            ric[0] = RIC[0];
            f_manager.setRic(ric);
            f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));
            //f_manager.triangulateWithDepth(Ps, &(TIC_TMP[0]), &(RIC[0]));

            return true;
        }
    }

#endif    

    TicToc t_sfm;
    //check imu observibility
    {
        Eigen::aligned_map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = localWindowFrames.begin(), frame_it++; frame_it != localWindowFrames.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int) localWindowFrames.size() - 1);
        double var = 0;
        for (frame_it = localWindowFrames.begin(), frame_it++; frame_it != localWindowFrames.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int) localWindowFrames.size() - 1));
        //RCLCPP_WARN(this->get_logger(), "IMU variation %f!", var);
        if (var < 0.25) {
            LOG(INFO) << "IMU excitation not enouth!";
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    Eigen::aligned_map<int, Vector3d> sfm_tracked_points;
    Eigen::aligned_vector<SFMFeature> sfm_f;

    // TODO: 使用 unordered_map 代替　list
    // TODO: 需要修改的地方

 //   LOG(INFO) << "COME HERE!";
   // LOG(INFO)<<"f_manager.KeyPointLandmarks: "<<f_manager.KeyPointLandmarks.size();
    for (auto &landmark : f_manager.KeyPointLandmarks) {
        SFMFeature tmp_feature;
        tmp_feature.state = false; // 表示没有深度
        tmp_feature.id = landmark.second.feature_id; // landmark的id号

        auto observation = landmark.second.obs;
 //       LOG(INFO)<<" landmark.second.obs.size(): "<< landmark.second.obs.size();
        for (const auto &obser_per_frame : landmark.second.obs) {

            auto frame_intid = time_frameid2_int_frameid.at(obser_per_frame.first);
         //   LOG(INFO)<<"frame_intid: "<<frame_intid;

            Vector3d pts_j = obser_per_frame.second.point;

            tmp_feature.observation.push_back(make_pair(frame_intid, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
            tmp_feature.observation_depth.emplace_back(frame_intid, obser_per_frame.second.depth_measured);
        }
        sfm_f.push_back(tmp_feature);
    }

   // LOG(INFO) << "Already Pass!"<<"num of sfm_f: "<<sfm_f.size();
   //assert(frame_count==1000);

    Matrix3d relative_R;
    Vector3d relative_T;
    int l;

    //保证具有足够的视差,由F矩阵恢复Rt
    //第l帧是从第一帧开始到滑动窗口中第一个满足与当前帧的平均视差足够大的帧，会作为参考帧到下面的全局sfm使用
    //此处的relative_R，relative_T为当前帧到参考帧（第l帧）的坐标系变换Rt
    if (!relativePose(relative_R, relative_T, l)) {
        LOG(INFO) << "Not enough features or parallax; Move device around";
        return false;
    }

    //对窗口中每个图像帧求解sfm问题
    //得到所有图像帧相对于参考帧的姿态四元数Q、平移向量T和特征点坐标sfm_tracked_points。
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points)) {
        LOG(ERROR) << "global SFM failed!";
        marginalization_flag = MarginalizationFlag::MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    //TODO: 为啥又要PNP一次？
    //对于所有的图像帧，包括不在滑动窗口中的，提供初始的RT估计，然后solvePnP进行优化,得到每一帧的姿态
    Eigen::aligned_map<double, ImageFrame>::iterator frame_it;
    Eigen::aligned_map<int, Vector3d>::iterator it;
    frame_it = localWindowFrames.begin();
    for (int i = 0; frame_it != localWindowFrames.end(); frame_it++) {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i].stamp.toSec()) {
            frame_it->second.is_key_frame = true;

            frame_it->second.Twi = Transformd(Q[i].toRotationMatrix() * RIC[0].transpose(), T[i]);
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i].stamp.toSec()) {
            i++;
        }
        //Q和T是图像帧的位姿，而不是求解PNP时所用的坐标系变换矩阵
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        //points: map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
        for (auto &id_pts : frame_it->second.points) {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second) {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end()) {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6) {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            LOG(INFO) << "Not enough points for solve pnp !";
            return false;
        }
        /**
 *bool cv::solvePnP(    求解pnp问题
 *   InputArray  objectPoints,   特征点的3D坐标数组
 *   InputArray  imagePoints,    特征点对应的图像坐标
 *   InputArray  cameraMatrix,   相机内参矩阵
 *   InputArray  distCoeffs,     失真系数的输入向量
 *   OutputArray     rvec,       旋转向量
 *   OutputArray     tvec,       平移向量
 *   bool    useExtrinsicGuess = false, 为真则使用提供的初始估计值
 *   int     flags = SOLVEPNP_ITERATIVE 采用LM优化
 *)
 */

        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, true)) {
            RCLCPP_DEBUG(this->get_logger(), "solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        //这里也同样需要将坐标变换矩阵转变成图像帧位姿，并转换为IMU坐标系的位姿
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);

        frame_it->second.Twi = Transformd(R_pnp * RIC[0].transpose(), T_pnp);
    }
    // Rs Ps ric init
    //进行视觉惯性联合初始化
#if 1   
    if(visualInitialAlign())
#else     
    if (visualInitialAlignWithDepth())
#endif    
        return true;
    else {
        RCLCPP_INFO(this->get_logger(),"misalign visual structure with IMU");
        return false;
    }

} //function initialStructure

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(localWindowFrames, Bgs, g, x);
    if (!result) {
        ROS_ERROR("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++) {

        Matrix3d Ri = localWindowFrames[Headers[i].stamp.toSec()].Twi.rotationMatrix();
        Vector3d Pi = localWindowFrames[Headers[i].stamp.toSec()].Twi.pos;

        Ps[i] = Pi;
        Rs[i] = Ri;
        localWindowFrames[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    f_manager.resetDepth();

    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));
    //f_manager.triangulateWithDepth(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    //RCLCPP_DEBUG(this->get_logger(), "the scale is %f\n", s);
    // do repropagate here
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    Eigen::aligned_map<double, ImageFrame>::iterator frame_i;
    for (frame_i = localWindowFrames.begin(); frame_i != localWindowFrames.end(); frame_i++) {
        if (frame_i->second.is_key_frame) {
            kv++;
            Vs[kv] = frame_i->second.Twi.rot * x.segment<3>(kv * 3);
        }
    }


    for (auto &landmark : f_manager.KeyPointLandmarks) {
        landmark.second.used_num = landmark.second.obs.size();
        if (!(landmark.second.used_num >= 2 && time_frameid2_int_frameid.at(landmark.second.kf_id) < WINDOW_SIZE - 2))
            continue;
        landmark.second.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++) {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

/**
 * @brief   视觉惯性联合初始化
 * @Description 陀螺仪的偏置校准(加速度偏置没有处理) 计算速度V[0:n] 重力g 尺度s
 *              更新了Bgs后，IMU测量量需要repropagate
 *              得到尺度s和重力g的方向后，需更新所有图像帧在世界坐标系下的Ps、Rs、Vs
 * @return  bool true：成功
 */

bool Estimator::visualInitialAlignWithDepth()
{
    TicToc t_g;
    VectorXd x;
    //solve scale

    //计算陀螺仪偏置，尺度，重力加速度和速度
    bool result = VisualIMUAlignment(localWindowFrames, Bgs, g, x);
    if (!result) {
        ROS_ERROR("solve g failed!");
        return false;
    }

    // change state
    // 得到所有图像帧的位姿Ps、Rs，并将其置为关键帧
    for (int i = 0; i <= frame_count; i++) {
        Matrix3d Ri = localWindowFrames[Headers[i].stamp.toSec()].Twi.rotationMatrix();
        Vector3d Pi = localWindowFrames[Headers[i].stamp.toSec()].Twi.pos;

        Ps[i] = Pi;
        Rs[i] = Ri;
        localWindowFrames[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    //将所有特征点的深度置为-1
    f_manager.resetDepth();

    //triangulat on cam pose , no tic
    //重新计算特征点的深度
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));
    //f_manager.triangulateWithDepth(Ps, &(TIC_TMP[0]), &(RIC[0]));


    //陀螺仪的偏置bgs改变，重新计算预积分
    // do repropagate here
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    //ROS_ERROR("before %f | %f | %f\n", Ps[1].x(), Ps[1].y(), Ps[1].z());//shan add
    //将Ps、Vs、depth 不用缩放尺度
    for (int i = frame_count; i >= 0; i--)
        //Ps转变为第i帧imu坐标系到第0帧imu坐标系的变换
        Ps[i] = Ps[i] - Rs[i] * TIC[0] - (Ps[0] - Rs[0] * TIC[0]);
    //ROS_ERROR("after  %f | %f | %f\n", Ps[1].x(), Ps[1].y(), Ps[1].z());//shan add
    int kv = -1;
    Eigen::aligned_map<double, ImageFrame>::iterator frame_i;
    for (frame_i = localWindowFrames.begin(); frame_i != localWindowFrames.end(); frame_i++) {
        if (frame_i->second.is_key_frame) {
            kv++;
            //Vs为优化得到的速度
            Vs[kv] = frame_i->second.Twi.rot * x.segment<3>(kv * 3);
        }
    }

    //通过将重力旋转到z轴上，得到世界坐标系与摄像机坐标系c0之间的旋转矩阵rot_diff
    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
     

    auto first_frame_tid = int_frameid2_time_frameid.rbegin()->second;
    auto first_frame_index = int_frameid2_time_frameid.rbegin()->first;
    Transformd T_i_meas_first = localWindowFrames.at(first_frame_tid).Tw_imu_meas;
    bool laser_vio_sync = localWindowFrames.at(first_frame_tid).laser_odom_vio_sync; 

#if 1    
      if (laser_vio_sync == true)
    {

        Matrix3d rot_diff = T_i_meas_first.rotationMatrix() * Rs[first_frame_index].transpose();
        Vector3d tran_diff = T_i_meas_first.pos - Ps[first_frame_index];

        //所有变量从参考坐标系c0旋转到世界坐标系w (IMU的坐标系)
        for (int i = 0; i <= frame_count; i++)
        {
            Ps[i] = rot_diff * Ps[i] + tran_diff; //shan 转到世界坐标系下？ 没明白
            Rs[i] = rot_diff * Rs[i];
            Vs[i] = rot_diff * Vs[i];
            //ROS_ERROR("%d farme's t is %f | %f | %f\n",i, Ps[i].x(), Ps[i].y(), Ps[i].z());//shan add
        }
        RCLCPP_INFO(this->get_logger(),"\033[success correct the first pose!.\033[0m");
        ROS_DEBUG_STREAM("g0     " << g.transpose());
        ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"\033[initialization first pose correction has something wrong!\033[0m");
    }

# else
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    //所有变量从参考坐标系c0旋转到世界坐标系w (IMU的坐标系)
    for (int i = 0; i <= frame_count; i++) {
        Ps[i] = rot_diff * Ps[i];//shan 转到世界坐标系下？ 没明白
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
        //ROS_ERROR("%d farme's t is %f | %f | %f\n",i, Ps[i].x(), Ps[i].y(), Ps[i].z());//shan add
    }
#endif

    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

/**
 * @brief   判断两帧有足够视差30且内点数目大于12则可进行初始化，同时得到R和T
 * @Description    判断每帧到窗口最后一帧对应特征点的平均视差是否大于30
                solveRelativeRT()通过基础矩阵计算当前帧与第l帧之间的R和T,并判断内点数目是否足够
 * @param[out]   relative_R 当前帧到第l帧之间的旋转矩阵R
 * @param[out]   relative_T 当前帧到第l帧之间的平移向量T
 * @param[out]   L 保存滑动窗口中与当前帧满足初始化条件的那一帧
 * @return  bool 1:可以进行初始化;0:不满足初始化条件
*/
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++) {
        Eigen::aligned_vector<pair<Vector3d, Vector3d>> corres;
        //corres = f_manager.getCorresponding(i, WINDOW_SIZE);

        //TODO：深度值参与初始化过程
        //寻找第i帧到窗口最后一帧的对应特征点
#if 0
        corres = f_manager.getCorrespondingWithDepth(i, WINDOW_SIZE);
#else
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
#endif
        if (corres.size() > 20) {
            ROS_INFO_STREAM("corres size: " << corres.size());
            //计算平均视差
            double sum_parallax = 0;
            double average_parallax;
            for (size_t j = 0; j < corres.size(); j++) {
                // 第j个对应点在第i帧和最后一帧的(x,y)
                Vector2d pts_0(corres[j].first(0) / corres[j].first(2), corres[j].first(1) / corres[j].first(2));
                Vector2d pts_1(corres[j].second(0) / corres[j].second(2), corres[j].second(1) / corres[j].second(2));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());

            RCLCPP_INFO(this->get_logger(),"average_parallax %f", average_parallax);

            //判断是否满足初始化条件：视差>30和内点数满足要求
            //同时返回窗口最后一帧（当前帧）到第l帧（参考帧）的Rt
#if 0
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT_PNP(corres, relative_R, relative_T)) {
                //                Matrix3d relative_R2; Vector3d relative_T2;
                //                m_estimator.solveRelativeRT(corres, relative_R2, relative_T2);
                l = i;
                RCLCPP_DEBUG(this->get_logger(), "average_parallax %f choose l %d and newest frame to triangulate the whole structure",
                          average_parallax * 460,
                          l);
                return true;
            }
#else
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
                //                Matrix3d relative_R2; Vector3d relative_T2;
                //                m_estimator.solveRelativeRT(corres, relative_R2, relative_T2);
                l = i;
                RCLCPP_DEBUG(this->get_logger(), "average_parallax %f choose l %d and newest frame to triangulate the whole structure",
                          average_parallax * 1490,
                          l);
                return true;
            }
#endif
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == SolverFlag::NON_LINEAR) {
        TicToc t_tri;
        f_manager.triangulateWithDepth(Ps, tic, ric);
        f_manager.triangulate(Ps, tic, ric);
//        LOG(INFO) << "triangulation costs: " << t_tri.toc();
        optimization();
        updateFramePose();

    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }

    // TODO: 3D点更新
    f_manager.depth2InvDepth();
    for (int i = 0; i < NUM_OF_CAM; i++) {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur) {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = false;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
        RCLCPP_DEBUG(this->get_logger(), "euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5]).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized()
            .toRotationMatrix();

        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;

        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    // TODO: 3D 点更新
    // ------------------
    f_manager.invDepth2Depth();
    //-------------------

    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // relative info between two loop frame
    if (relocalization_info) {
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff
            * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) + origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw =
            Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = false;

    }
}

void Estimator::updateFramePose()
{
    for (size_t i = 0; i < WINDOW_SIZE + 1; i++) {
        auto timestamp = Headers[i].stamp.toSec();
        if (localWindowFrames.find(timestamp) != localWindowFrames.end()) {
            localWindowFrames[timestamp].Twi.rot = Rs[i];
            localWindowFrames[timestamp].Twi.pos = Ps[i];
        }
    }
}
bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2) {
        ROS_ERROR(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5) {
        ROS_ERROR(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0) {
        ROS_ERROR(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
      // TODO (henryzh47): change to 6m/s
    if (Vs[WINDOW_SIZE].norm() > 10.0)
    {
        ROS_ERROR("VINS big speed %f, restart estimator!", Vs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        RCLCPP_INFO(this->get_logger()," big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 20) {
        RCLCPP_ERROR(this->get_logger()," big translation: " << (tmp_P - last_P).norm());
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1.5) {
        RCLCPP_ERROR(this->get_logger()," big z translation: " << abs(tmp_P.z() - last_P.z()));
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50) {
        ROS_ERROR(" big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC) {
      //      LOG(INFO) << "fix extinsic param";
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            LOG(INFO) << "estimate extinsic param";
    }
    if (ESTIMATE_TD) {
        problem.AddParameterBlock(para_Td[0], 1);
        //problem.SetParameterBlockConstant(para_Td[0]);
    }

    TicToc t_whole, t_prepare;
    vector2double();

#ifdef VISUAL_IMU_SUM_PRIOR
    if (last_marginalization_info) {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, nullptr,
                                 last_marginalization_parameter_blocks);
    }

#else
    if (last_visual_marginalization_info) {
        MarginalizationFactor
            *visual_marginalization_factor = new MarginalizationFactor(last_visual_marginalization_info);
        problem.AddResidualBlock(visual_marginalization_factor, nullptr,
                                 last_visual_marginalization_parameter_blocks);
        LOG(INFO) << "[visual_marginalization_parameter_blocks] size: " << last_visual_marginalization_parameter_blocks.size();
    }

    if (last_imu_marginalization_info) {
        MarginalizationFactor *imu_marginalization_factor = new MarginalizationFactor(last_imu_marginalization_info);
        problem.AddResidualBlock(imu_marginalization_factor, nullptr,
                                 last_imu_marginalization_parameter_blocks);

        LOG(INFO) << "[imu_marginalization_parameter_blocks] size: " << last_imu_marginalization_parameter_blocks.size();
    }
#endif

    for (int i = 0; i < WINDOW_SIZE; i++) {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, nullptr, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    int f_m_cnt = 0;
    
       // TODO: 增加激光的相对因子
    if (USE_LIDAR_ODOM_MEAS)
    {
        Eigen::Mat66d information;
        information.setIdentity();
        information(0, 0) = 100 *SCALE_LIDAR_ABSOLUTE_FACTOR;
        information(1, 1) = 100 *SCALE_LIDAR_ABSOLUTE_FACTOR;
        information(2, 2) = 100 *SCALE_LIDAR_ABSOLUTE_FACTOR;

        information(3, 3) = 100;
        information(4, 4) = 100;
        information(5, 5) = 100 *SCALE_LIDAR_ABSOLUTE_FACTOR;

        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            int j = i + 1;

            auto frame_i_tid = int_frameid2_time_frameid.at(i);
            auto frame_j_tid = int_frameid2_time_frameid.at(j);
            Transformd T_i_meas = localWindowFrames.at(frame_i_tid).Tw_imu_meas;
            Transformd T_j_meas = localWindowFrames.at(frame_j_tid).Tw_imu_meas;

            bool laser_vio_sync = localWindowFrames.at(frame_i_tid).laser_odom_vio_sync;
           // LOG(INFO)<<"lidar status: "<<localWindowFrames.at(frame_i_tid).reset_id;

            //      Transformd T_i_j_meas = T_i_meas.inverse() * T_j_meas;
            //      SE3RelativtePoseFactor *relativtePoseFactor =
            //          new SE3RelativtePoseFactor(T_i_j_meas, information);
            //      problem.AddResidualBlock(relativtePoseFactor, nullptr,
            //      para_Pose[i],
            //                               para_Pose[j]);
            if (laser_vio_sync == true and localWindowFrames.at(frame_i_tid).reset_id==1 and
                     localWindowFrames.at(frame_j_tid).reset_id==1)
            {
              //  RCLCPP_INFO(this->get_logger(),"\033[1;32m----> ADD Absolute factor.\033[0m");
#if USE_ABSOLUTE_FACTOR            
                Transformd T_i_j_meas = T_i_meas.inverse() * T_j_meas;

                Eigen::Vector3d Pi(para_Pose[i][0],para_Pose[i][1], para_Pose[i][2]);
                
                Eigen::Quaterniond Qi( para_Pose[i][6], para_Pose[i][3],  para_Pose[i][4],
                         para_Pose[i][5]);
                
                Transformd T_w_i_estimate(Qi, Pi);

                T_i_meas=T_w_i_estimate* T_i_j_meas;

                SE3AbsolutatePoseFactor *absolutatePoseFactor =
                    new SE3AbsolutatePoseFactor(T_i_meas, information);
                problem.AddResidualBlock(absolutatePoseFactor, nullptr, para_Pose[i]);

               // RCLCPP_WARN(this->get_logger(), "\033[1;32m----> ADD Laser Absolute factor .\033[0m");

#else
             
             if(localWindowFrames.at(frame_j_tid).reset_id==1)
             {  
               
               Transformd T_i_j_meas = T_i_meas.inverse() * T_j_meas;
               SE3RelativtePoseFactor *relativtePoseFactor = new SE3RelativtePoseFactor(T_i_j_meas, information);
               problem.AddResidualBlock(relativtePoseFactor, nullptr, para_Pose[i],para_Pose[j]);
             
             }    


#endif

            }else
            {
             //   RCLCPP_INFO(this->get_logger(),"\033[1;32m----> NOT ADD Absolute factor Because LASER DEGRADED.\033[0m");

            }
        }
    }

    // TODO: 使用 unordered map 代替　list

    for (auto &landmark : f_manager.KeyPointLandmarks) {
        landmark.second.used_num = landmark.second.obs.size();
        if (!(landmark.second.used_num >= 2 &&
            time_frameid2_int_frameid.at(landmark.second.kf_id) < WINDOW_SIZE - 2))
            continue;

        auto host_tid = landmark.second.kf_id; // 第一个观测值对应的就是主导真
        int host_id = time_frameid2_int_frameid.at(host_tid);
        const auto &obs = landmark.second.obs;
        Vector3d pts_i = obs.at(host_tid).point;

        for (const auto &it_per_frame : obs) {
            auto target_tid = it_per_frame.first;
            int target_id = time_frameid2_int_frameid.at(target_tid);

            if (host_tid == target_tid)
                continue;

            Vector3d pts_j = it_per_frame.second.point;
            if (ESTIMATE_TD) {
                ProjectionTdFactor *f_td =
                    new ProjectionTdFactor(pts_i,
                                           pts_j,
                                           obs.at(host_tid).velocity,
                                           it_per_frame.second.velocity,
                                           obs.at(host_tid).cur_td,
                                           it_per_frame.second.cur_td,
                                           obs.at(host_tid).uv.y(),
                                           it_per_frame.second.uv.y());
                problem.AddResidualBlock(f_td,
                                         loss_function,
                                         para_Pose[host_id],
                                         para_Pose[target_id],
                                         para_Ex_Pose[0],
                                         landmark.second.data.data(),
                                         para_Td[0]);

                if (landmark.second.estimate_flag == KeyPointLandmark::EstimateFlag::DIRECT_MEASURED)
                    problem.SetParameterBlockConstant(landmark.second.data.data());

            }
            else {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f,
                                         loss_function,
                                         para_Pose[host_id],
                                         para_Pose[target_id],
                                         para_Ex_Pose[0],
                                         landmark.second.data.data());

                if (landmark.second.estimate_flag == KeyPointLandmark::EstimateFlag::DIRECT_MEASURED)
                    problem.SetParameterBlockConstant(landmark.second.data.data());
            }
            f_m_cnt++;
        }
    }
    // TODO: 使用 unordered map 代替　list

    RCLCPP_DEBUG(this->get_logger(), "visual measurement count: %d", f_m_cnt);
    RCLCPP_DEBUG(this->get_logger(), "prepare for ceres: %f", t_prepare.toc());

    if (relocalization_info) {
        //printf("set relocalization factor! \n");
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);

        // TODO: 使用 unordered map 代替　list
        for (auto &match_point : match_points) {
            int feature_id = match_point.z();

            // 查看landmarks数据库中能否找到id号相同的landmark
            if (f_manager.KeyPointLandmarks.find(feature_id) != f_manager.KeyPointLandmarks.end()) {
                auto &landmark = f_manager.KeyPointLandmarks.at(feature_id);

                // 确定landmark是否合法
                landmark.used_num = landmark.obs.size();
                if (!(landmark.used_num >= 2
                    and time_frameid2_int_frameid.at(landmark.kf_id) < WINDOW_SIZE - 2)) {
                    continue;
                }

                // 如果landmark合法
                auto host_tid = landmark.kf_id;
                int host_id = time_frameid2_int_frameid.at(host_tid);

                Vector3d pts_i = landmark.obs.at(host_tid).point;
                Vector3d pts_j = match_point;
                pts_j[2] = 1.0;

                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);

                // TODO: 感觉需要把逆深度优化的写入到landmark中
                // -------------------------------------
                problem.AddResidualBlock(f,
                                         loss_function,
                                         para_Pose[host_id],
                                         relo_Pose,
                                         para_Ex_Pose[0],
                                         landmark.data.data());
                // ------------------------------------
            }
        }
        // TODO: 使用 unordered map 代替　list
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MarginalizationFlag::MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    RCLCPP_DEBUG(this->get_logger(), "Iterations : %d", static_cast<int>(summary.iterations.size()));
    RCLCPP_DEBUG(this->get_logger(), "solver costs: %f", t_solver.toc());

    double2vector();

    TicToc t_whole_marginalization;
#ifdef VISUAL_IMU_SUM_PRIOR
    if (marginalization_flag == MarginalizationFlag::MARGIN_OLD) {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info) {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, nullptr,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0) {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, nullptr,
                                                                               vector<double *>{para_Pose[0],
                                                                                                para_SpeedBias[0],
                                                                                                para_Pose[1],
                                                                                                para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            // TODO: 使用 unordered map 代替　list
            for (auto &landmark : f_manager.KeyPointLandmarks) {
                landmark.second.used_num = landmark.second.obs.size();
                if (!(landmark.second.used_num >= 2
                    && time_frameid2_int_frameid.at(landmark.second.kf_id) < WINDOW_SIZE - 2))
                    continue;

                auto host_tid = landmark.second.kf_id;
                int host_id = time_frameid2_int_frameid.at(host_tid);

                if (host_id != 0)
                    continue;

                const auto &obs = landmark.second.obs;

                Vector3d pts_i = obs.at(host_tid).point;

                for (const auto &it_per_frame : obs) {

                    auto target_tid = it_per_frame.first;
                    int target_id = time_frameid2_int_frameid.at(target_tid);

                    if (host_tid == target_tid)
                        continue;

                    Vector3d pts_j = it_per_frame.second.point;

                    if (ESTIMATE_TD) {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i,
                                                                          pts_j,
                                                                          obs.at(host_tid).velocity,
                                                                          it_per_frame.second.velocity,
                                                                          obs.at(host_tid).cur_td,
                                                                          it_per_frame.second.cur_td,
                                                                          obs.at(host_tid).uv.y(),
                                                                          it_per_frame.second.uv.y());

                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{
                                                                                           para_Pose[host_id],
                                                                                           para_Pose[target_id],
                                                                                           para_Ex_Pose[0],
                                                                                           landmark.second.data.data(),
                                                                                           para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{
                                                                                           para_Pose[host_id],
                                                                                           para_Pose[target_id],
                                                                                           para_Ex_Pose[0],
                                                                                           landmark.second.data.data()},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        RCLCPP_DEBUG(this->get_logger(), "pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        RCLCPP_DEBUG(this->get_logger(), "marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++) {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD) {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;

    }
    else {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks),
                       std::end(last_marginalization_parameter_blocks),
                       para_Pose[WINDOW_SIZE - 1])) {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info) {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, nullptr,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            RCLCPP_DEBUG(this->get_logger(), "begin marginalization");
            marginalization_info->preMarginalize();
            RCLCPP_DEBUG(this->get_logger(), "end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            RCLCPP_DEBUG(this->get_logger(), "begin marginalization");
            marginalization_info->marginalize();
            RCLCPP_DEBUG(this->get_logger(), "end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++) {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE) {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD) {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

        }
    }
#else
    if (marginalization_flag == MARGIN_OLD) {
        MarginalizationInfo *visual_marginalization_info = new MarginalizationInfo();
        MarginalizationInfo *imu_marginalization_info = new MarginalizationInfo();

        vector2double();
        if (last_visual_marginalization_info) {
            LOG(INFO) << "last have visual prior";
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_visual_marginalization_parameter_blocks.size()); i++) {
                if (last_visual_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_visual_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor
                *visual_marginalization_factor = new MarginalizationFactor(last_visual_marginalization_info);
            ResidualBlockInfo
                *viusal_residual_block_info = new ResidualBlockInfo(visual_marginalization_factor, nullptr,
                                                                    last_visual_marginalization_parameter_blocks,
                                                                    drop_set);
            visual_marginalization_info->addResidualBlockInfo(viusal_residual_block_info);
        }
        if (last_imu_marginalization_info) {
            LOG(INFO) << "last have imu prior";
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_imu_marginalization_parameter_blocks.size()); i++) {
                if (last_imu_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_imu_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor
                *imu_marginalization_factor = new MarginalizationFactor(last_imu_marginalization_info);
            ResidualBlockInfo
                *imu_residual_block_info = new ResidualBlockInfo(imu_marginalization_factor, nullptr,
                                                                 last_imu_marginalization_parameter_blocks,
                                                                 drop_set);
            imu_marginalization_info->addResidualBlockInfo(imu_residual_block_info);
        }

        {
            // 添加要边缘化的IMU因子
            // TODO: 应该设置一个标志,判断预积分是否合理
            if (pre_integrations[1]->sum_dt < 10.0) {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, nullptr,
                                                                               vector<double *>{para_Pose[0],
                                                                                                para_SpeedBias[0],
                                                                                                para_Pose[1],
                                                                                                para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                imu_marginalization_info->addResidualBlockInfo(residual_block_info);
                LOG(INFO) << "marginalize old frame imu factor";
            }
        }

        {
            // 添加视觉因子
            int feature_index = -1;

            // TODO: 使用 unordered_map 代替　list
            for (auto &landmark : f_manager.landmarkDatabase) {
                landmark.second.used_num = landmark.second.feature_per_frame.size();
                if (!(landmark.second.used_num >= 2 && landmark.second.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = landmark.second.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = landmark.second.feature_per_frame[0].point;

                for (auto &it_per_frame : landmark.second.feature_per_frame) {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD) {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i,
                                                                          pts_j,
                                                                          landmark.second.feature_per_frame[0].velocity,
                                                                          it_per_frame.velocity,
                                                                          landmark.second.feature_per_frame[0].cur_td,
                                                                          it_per_frame.cur_td,
                                                                          landmark.second.feature_per_frame[0].uv.y(),
                                                                          it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{
                                                                                           para_Pose[imu_i],
                                                                                           para_Pose[imu_j],
                                                                                           para_Ex_Pose[0],
                                                                                           para_Feature[feature_index],
                                                                                           para_Td[0]},
                                                                                       vector<int>{0, 3});
                        visual_marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{
                                                                                           para_Pose[imu_i],
                                                                                           para_Pose[imu_j],
                                                                                           para_Ex_Pose[0],
                                                                                           para_Feature[feature_index]},
                                                                                       vector<int>{0, 3});
                        visual_marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
            // TODO: 使用 unordered map 代替　list
        }

        TicToc t_pre_margin;
        visual_marginalization_info->preMarginalize();
        imu_marginalization_info->preMarginalize();
        LOG(INFO) << "pre visual and imu marginalization " << t_pre_margin.toc() << "ms";

        TicToc t_margin;
        visual_marginalization_info->marginalize();
        imu_marginalization_info->marginalize();
        LOG(INFO) << "marginalization visual and imu " << t_margin.toc() << "ms";

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++) {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD) {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }

        vector<double *> visual_parameter_blocks = visual_marginalization_info->getParameterBlocks(addr_shift);
        vector<double *> imu_parameter_blocks = imu_marginalization_info->getParameterBlocks(addr_shift);

        // 删除空指针没事
        if (last_visual_marginalization_info)
            delete last_visual_marginalization_info;

        if (last_imu_marginalization_info)
            delete last_imu_marginalization_info;

        last_visual_marginalization_info = visual_marginalization_info;
        last_visual_marginalization_parameter_blocks = visual_parameter_blocks;

        last_imu_marginalization_info = imu_marginalization_info;
        last_imu_marginalization_parameter_blocks = imu_parameter_blocks;
    }
    else {
        LOG(INFO) << "marginalization second newest frame start";
        // 视觉先验的构造
        if (last_visual_marginalization_info and
            std::count(std::begin(last_visual_marginalization_parameter_blocks),
                       std::end(last_visual_marginalization_parameter_blocks),
                       para_Pose[WINDOW_SIZE - 1])
            ) {

            MarginalizationInfo *visual_marginalization_info = new MarginalizationInfo();
            vector2double();

            if (last_visual_marginalization_info) {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_visual_marginalization_parameter_blocks.size()); i++) {
                    ROS_ASSERT(last_visual_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_visual_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor
                    *visual_marginalization_factor = new MarginalizationFactor(last_visual_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(visual_marginalization_factor,
                                                                               nullptr,
                                                                               last_visual_marginalization_parameter_blocks,
                                                                               drop_set);

                visual_marginalization_info->addResidualBlockInfo(residual_block_info);


                TicToc t_pre_margin;
                RCLCPP_DEBUG(this->get_logger(), "begin visual marginalization");
                visual_marginalization_info->preMarginalize();
                RCLCPP_DEBUG(this->get_logger(), "end pre visual marginalization, %f ms", t_pre_margin.toc());

                TicToc t_margin;
                RCLCPP_DEBUG(this->get_logger(), "begin visual marginalization");
                visual_marginalization_info->marginalize();
                RCLCPP_DEBUG(this->get_logger(), "end viusal marginalization, %f ms", t_margin.toc());

                std::unordered_map<long, double *> addr_shift;
                for (int i = 0; i <= WINDOW_SIZE; i++) {
                    if (i == WINDOW_SIZE - 1)
                        continue;
                    else if (i == WINDOW_SIZE) {
                        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                    }
                    else {
                        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                    }
                }
                for (int i = 0; i < NUM_OF_CAM; i++)
                    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
                if (ESTIMATE_TD) {
                    addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
                }

                vector<double *> parameter_blocks = visual_marginalization_info->getParameterBlocks(addr_shift);
                if (last_visual_marginalization_info)
                    delete last_visual_marginalization_info;
                last_visual_marginalization_info = visual_marginalization_info;
                last_visual_marginalization_parameter_blocks = parameter_blocks;
            }
        } // 构造世界的先验 end

        // IMU先验的构造
        if (last_imu_marginalization_info and
            std::count(std::begin(last_imu_marginalization_parameter_blocks),
                       std::end(last_imu_marginalization_parameter_blocks),
                       para_Pose[WINDOW_SIZE - 1])
            ) {

            MarginalizationInfo *imu_marginalization_info = new MarginalizationInfo();
            vector2double();

            if (last_imu_marginalization_info) {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_imu_marginalization_parameter_blocks.size()); i++) {
                    ROS_ASSERT(last_imu_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_imu_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor
                    *imu_marginalization_factor = new MarginalizationFactor(last_imu_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_marginalization_factor,
                                                                               nullptr,
                                                                               last_imu_marginalization_parameter_blocks,
                                                                               drop_set);

                imu_marginalization_info->addResidualBlockInfo(residual_block_info);

                TicToc t_pre_margin;
                RCLCPP_DEBUG(this->get_logger(), "begin imu marginalization");
                imu_marginalization_info->preMarginalize();
                RCLCPP_DEBUG(this->get_logger(), "end pre imu marginalization, %f ms", t_pre_margin.toc());

                TicToc t_margin;
                RCLCPP_DEBUG(this->get_logger(), "begin imu marginalization");
                imu_marginalization_info->marginalize();
                RCLCPP_DEBUG(this->get_logger(), "end imu marginalization, %f ms", t_margin.toc());

                std::unordered_map<long, double *> addr_shift;
                for (int i = 0; i <= WINDOW_SIZE; i++) {
                    if (i == WINDOW_SIZE - 1)
                        continue;
                    else if (i == WINDOW_SIZE) {
                        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                    }
                    else {
                        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                    }
                }
                for (int i = 0; i < NUM_OF_CAM; i++)
                    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
                if (ESTIMATE_TD) {
                    addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
                }

                vector<double *> parameter_blocks = imu_marginalization_info->getParameterBlocks(addr_shift);
                if (last_imu_marginalization_info)
                    delete last_imu_marginalization_info;
                last_imu_marginalization_info = imu_marginalization_info;
                last_imu_marginalization_parameter_blocks = parameter_blocks;
            }
        } // 构造IMU的先验 end
        LOG(INFO) << "marginalization second newest frame start";
    }
#endif
//    LOG(INFO) << "whole marginalization costs: " << t_whole_marginalization.toc();

//    LOG(INFO) << "whole time for ceres: " << t_whole.toc();
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MarginalizationFlag::MARGIN_OLD) {

     
        // TODO: 删除关键帧
        local_active_frames.erase(int_frameid2_time_frameid[0]);

    
        double t_0 = Headers[0].stamp.toSec();
        if (frame_count == WINDOW_SIZE) {

            for (int i = 0; i < WINDOW_SIZE; i++) {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            Eigen::aligned_map<double, ImageFrame>::iterator it_0;
            // TODO: 这里会移动滑窗
            if(true||solver_flag == SolverFlag::INITIAL)
           { 
            it_0 = localWindowFrames.find(t_0);
            delete it_0->second.pre_integration;
            it_0->second.pre_integration = nullptr;
            for (auto it = localWindowFrames.begin(); it != it_0; ++it) {
                if (it->second.pre_integration)
                    delete it->second.pre_integration;
                it->second.pre_integration = nullptr;
            }
            localWindowFrames.erase(localWindowFrames.begin(), it_0);
            localWindowFrames.erase(t_0);

           
           }
         
      
        slideWindowOld();
     
        }
    }
    else {

  //      LOG(INFO) << "marginal new second frame begin";

        if (frame_count == WINDOW_SIZE) {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();
            
            // 删除次新帧
            local_active_frames.erase(int_frameid2_time_frameid[WINDOW_SIZE-1]);
            slideWindowNew();
   //         LOG(INFO) << "marginal new second frame end";
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    // 剔除次新帧的观测
    f_manager.removeOneFrameObservation(int_frameid2_time_frameid[WINDOW_SIZE-1]);

}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = (solver_flag == SolverFlag::NON_LINEAR);
 //   LOG(INFO) << "shift_depth: " << (shift_depth ? "true" : "false");
    if (shift_depth) {
        f_manager.removeOneFrameObservationAndShiftDepth(int_frameid2_time_frameid[0],
                                                         Ps,
                                                         tic, ric);
    }
    else
        f_manager.removeOneFrameObservation(int_frameid2_time_frameid[0]);
}

void Estimator::setReloFrame(double _frame_stamp,
                             int _frame_index,
                             Eigen::aligned_vector<Vector3d> &_match_points,
                             const Vector3d &_relo_t,
                             const Matrix3d &_relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        if (relo_frame_stamp == Headers[i].stamp.toSec()) {
            relo_frame_local_index = i;
            relocalization_info = true;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}
