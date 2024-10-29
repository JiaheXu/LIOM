#include "initial_sfm.h"

GlobalSFM::GlobalSFM()
{}

void GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                                 Eigen::Matrix<double, 3, 4> &Pose1,
                                 Vector2d &point0, Vector2d &point1,
                                 Vector3d &point_3d)
{
    Matrix4d design_matrix = Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Vector4d triangulated_point;
    triangulated_point =
        design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

bool GlobalSFM::solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i,
                                Eigen::aligned_vector<SFMFeature> &sfm_f)
{
    vector<cv::Point2f> pts_2_vector;
    vector<cv::Point3f> pts_3_vector;
    //对所有的特征进行遍历
    for (int j = 0; j < feature_num; j++) {   //如果该特征没有完成三角化
        if (sfm_f[j].state != true)
            continue;
        Vector2d point2d;
        //遍历所有观测到该特征的图像帧，以及找到该特征对应的图像坐标
        for (int k = 0; k < (int) sfm_f[j].observation.size(); k++) {
            //如果观测到该图像帧的图像ID中包含i帧
            if (sfm_f[j].observation[k].first == i) {
                //取出i对应的图像坐标
                Vector2d img_pts = sfm_f[j].observation[k].second;
                cv::Point2f pts_2(img_pts(0), img_pts(1));
                pts_2_vector.push_back(pts_2);

                //取出该特征对应的唯一空间3D点
                cv::Point3f pts_3(sfm_f[j].position[0], sfm_f[j].position[1], sfm_f[j].position[2]);
                pts_3_vector.push_back(pts_3);
                break;
            }
        }
    }
    if (int(pts_2_vector.size()) < 15) //如果约束少于15
    {
        printf("unstable features tracking, please slowly move you device!\n");
        if (int(pts_2_vector.size()) < 10)
            return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    //TODO：这里的K为啥是这样？
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool pnp_succ;
    //PNP 通过3D-2D匹配完成位姿计算
    pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
    if (!pnp_succ) {
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    R_initial = R_pnp;
    P_initial = T_pnp;
    return true;

}

void GlobalSFM::triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                                     int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                                     Eigen::aligned_vector<SFMFeature> &sfm_f)
{
    assert(frame0 != frame1);
    for (int j = 0; j < feature_num; j++) {
        if (sfm_f[j].state == true)
            continue;
        bool has_0 = false, has_1 = false;
        Vector2d point0;
        Vector2d point1;
        for (int k = 0; k < (int) sfm_f[j].observation.size(); k++) {
            if (sfm_f[j].observation[k].first == frame0) {
                point0 = sfm_f[j].observation[k].second;
                has_0 = true;
            }
            if (sfm_f[j].observation[k].first == frame1) {
                point1 = sfm_f[j].observation[k].second;
                has_1 = true;
            }
        }
        if (has_0 && has_1) {
            Vector3d point_3d;
            triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            sfm_f[j].state = true;
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
            //cout << "trangulated : " << frame1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }
    }
}

// 直接用带深度的特征点完成三角化
void GlobalSFM::triangulateTwoFramesWithDepth(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                                              int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                                              Eigen::aligned_vector<SFMFeature> &sfm_f)
{
    assert(frame0 != frame1);
    Matrix3d Pose0_R = Pose0.block<3, 3>(0, 0);
    Matrix3d Pose1_R = Pose1.block<3, 3>(0, 0);
    Vector3d Pose0_t = Pose0.block<3, 1>(0, 3);
    Vector3d Pose1_t = Pose1.block<3, 1>(0, 3);
    // 遍历所有的特征点
    for (int j = 0; j < feature_num; j++) {
        //检查是否完成三角化
        if (sfm_f[j].state)
            continue;
        bool has_0 = false, has_1 = false;
        bool depth0_valid = true;
        bool depth1_valid = true;
        Vector3d point0;
        Vector3d point1;

        Vector2d point00;
        Vector2d point01;
        //遍历共视该特征的图像帧
        for (int k = 0; k < (int) sfm_f[j].observation.size(); k++) {
            //如果该特征对应的共视帧中有l+1----最后一帧
            if (sfm_f[j].observation[k].first == frame0) {
                //两个条件筛选，就可以找到带深度的特征点--》恢复成空间3D点
                if (sfm_f[j].observation_depth[k].second < 1 || sfm_f[j].observation_depth[k].second > 120)
                    depth0_valid = false;
                if (depth0_valid) {
                    point0 = Vector3d(sfm_f[j].observation[k].second.x() * sfm_f[j].observation_depth[k].second,
                                      sfm_f[j].observation[k].second.y() * sfm_f[j].observation_depth[k].second,
                                      sfm_f[j].observation_depth[k].second);
                }
                point00 = sfm_f[j].observation[k].second;
                has_0 = true;

            }
            //如果该特征点的共视帧中有当前帧
            if (sfm_f[j].observation[k].first == frame1) {
                //找到当前帧中图像坐标
                if (sfm_f[j].observation_depth[k].second < 1 || sfm_f[j].observation_depth[k].second > 120)
                    depth1_valid = false;
                if (depth1_valid) {
                    point1 = Vector3d(sfm_f[j].observation[k].second.x() * sfm_f[j].observation_depth[k].second,
                                      sfm_f[j].observation[k].second.y() * sfm_f[j].observation_depth[k].second,
                                      sfm_f[j].observation_depth[k].second);
                }
                point01 = sfm_f[j].observation[k].second;
                has_1 = true;

            }

        }


        if (has_0 && has_1 && depth0_valid && depth1_valid) {
            Vector2d residual;
            Vector3d point_3d, point1_reprojected;
            //triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            //TODO：直接将深度值赋值以后，转换到世界坐标系即可
            // P_l_l+1=R_l+1_l.inverse()*P - R_l+1_l.inverse()*T_l+1_l
            point_3d = Pose0_R.transpose() * point0 - Pose0_R.transpose() * Pose0_t;//shan add:this is point in world;
            // P_cur_l+1=R_cur_l* P_l_l+1+T_cur_l
            point1_reprojected = Pose1_R * point_3d + Pose1_t;

            //将特征点转换到当前帧，然后计算图像像素误差
            residual = point01 - Vector2d(point1_reprojected.x() / point1_reprojected.z(),
                                          point1_reprojected.y() / point1_reprojected.z());

            //std::cout << residual.transpose()<<"norm"<<residual.norm()*460<<endl;
            //TODO：为什么是460
            if (residual.norm() < 1.0 / 460) {
                sfm_f[j].state = true;
                //空间的3D特征点都在第l帧坐标系下，也就是初始化成功的第一帧
                sfm_f[j].position[0] = point_3d(0);
                sfm_f[j].position[1] = point_3d(1);
                sfm_f[j].position[2] = point_3d(2);
            }
            //cout << "trangulated : " << frame1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }
        else if (has_0 && has_1 && depth0_valid) {
            Vector3d point_3d;

            point_3d = Pose0_R.transpose() * point0 - Pose0_R.transpose() * Pose0_t;//shan add:this is point in world;
            point_3d = Pose1_R * point_3d + Pose1_t;

            Vector2d residual;
            residual = point01 - Vector2d(point_3d.x() / point_3d.z(),
                                          point_3d.y() / point_3d.z());

            if (residual.norm() < 1.0 / 460) {
                sfm_f[j].state = true;
                //空间的3D特征点都在第l帧坐标系下，也就是初始化成功的第一帧
                sfm_f[j].position[0] = point_3d(0);
                sfm_f[j].position[1] = point_3d(1);
                sfm_f[j].position[2] = point_3d(2);
            }
        }
        else if (has_0 && has_1 && depth1_valid) {
            Vector3d point_3d, point1_reprojected;

            point_3d = Pose1_R.transpose() * point1 - Pose1_R.transpose() * Pose1_t;//shan add:this is point in world;
            point_3d = Pose0_R * point_3d + Pose0_t;

            Vector2d residual;
            residual = point00 - Vector2d(point_3d.x() / point_3d.z(),
                                          point_3d.y() / point_3d.z());

            if (residual.norm() < 1.0 / 460) {
                sfm_f[j].state = true;
                //空间的3D特征点都在第l帧坐标系下，也就是初始化成功的第一帧
                sfm_f[j].position[0] = point_3d(0);
                sfm_f[j].position[1] = point_3d(1);
                sfm_f[j].position[2] = point_3d(2);
            }
        }
        if (has_0 && has_1 && !sfm_f[j].state) {
            Vector3d point_3d;
            triangulatePoint(Pose0, Pose1, point00, point01, point_3d);
            sfm_f[j].state = true;
            sfm_f[j].position[0] = point_3d(0);
            sfm_f[j].position[1] = point_3d(1);
            sfm_f[j].position[2] = point_3d(2);
        }
    }
}

// 	 q w_R_cam t w_R_cam
//  c_rotation cam_R_w 
//  c_translation cam_R_w
// relative_q[i][j]  j_q_i
// relative_t[i][j]  j_t_ji  (j < i)



/**
 * @brief   纯视觉sfm，求解窗口中的所有图像帧的位姿和特征点坐标
 * @param[in]   frame_num	窗口总帧数（frame_count + 1）
 * @param[out]  q 	窗口内图像帧的旋转四元数q（相对于第l帧）
 * @param[out]	T 	窗口内图像帧的平移向量T（相对于第l帧）
 * @param[in]  	l 	第l帧
 * @param[in]  	relative_R	当前帧到第l帧的旋转矩阵
 * @param[in]  	relative_T 	当前帧到第l帧的平移向量
 * @param[in]  	sfm_f		所有特征点
 * @param[out]  sfm_tracked_points 所有在sfm中三角化的特征点ID和坐标
 * @return  bool true:sfm求解成功
**/
bool GlobalSFM::construct(int frame_num, Quaterniond *q, Vector3d *T, int l,
                          const Matrix3d relative_R, const Vector3d relative_T,
                          Eigen::aligned_vector<SFMFeature> &sfm_f,
                          Eigen::aligned_map<int, Vector3d> &sfm_tracked_points)
{
    feature_num = sfm_f.size();
    //cout << "set 0 and " << l << " as known " << endl;
    // have relative_r relative_t
    // intial two view

    //假设第l帧为原点，根据当前帧到第l帧的relative_R，relative_T，得到当前帧位姿
    q[l].w() = 1;
    q[l].x() = 0;
    q[l].y() = 0;
    q[l].z() = 0;
    T[l].setZero();
    q[frame_num - 1] = q[l] * Quaterniond(relative_R);
    T[frame_num - 1] = relative_T;
    //cout << "init q_l " << q[l].w() << " " << q[l].vec().transpose() << endl;
    //cout << "init t_l " << T[l].transpose() << endl;

    //rotate to cam frame
    Matrix3d c_Rotation[frame_num];
    Vector3d c_Translation[frame_num];
    Quaterniond c_Quat[frame_num];
    double c_rotation[frame_num][4];
    double c_translation[frame_num][3];
    Eigen::Matrix<double, 3, 4> Pose[frame_num];

    //这里的pose表示的是第l帧到每一帧的变换矩阵
    c_Quat[l] = q[l].inverse();
    c_Rotation[l] = c_Quat[l].toRotationMatrix();
    c_Translation[l] = -1 * (c_Rotation[l] * T[l]);
    Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
    Pose[l].block<3, 1>(0, 3) = c_Translation[l];

    c_Quat[frame_num - 1] = q[frame_num - 1].inverse();
    c_Rotation[frame_num - 1] = c_Quat[frame_num - 1].toRotationMatrix();
    c_Translation[frame_num - 1] = -1 * (c_Rotation[frame_num - 1] * T[frame_num - 1]);
    Pose[frame_num - 1].block<3, 3>(0, 0) = c_Rotation[frame_num - 1];
    Pose[frame_num - 1].block<3, 1>(0, 3) = c_Translation[frame_num - 1];


    //1: trangulate between l ----- frame_num - 1
    //2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1;

    //1、先三角化第l帧（参考帧）与第frame_num-1帧（当前帧）的路标点
    //TODO： 2、pnp求解从第l+1开始的每一帧到 最后一帧率的变换矩阵R_initial, P_initial，保存在Pose中，并与当前帧进行三角化 --》 是最后一帧吗？
    for (int i = l; i < frame_num - 1; i++) {
        // solve pnp
        if (i > l) {
            Matrix3d R_initial = c_Rotation[i - 1];
            Vector3d P_initial = c_Translation[i - 1];
            if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
                return false;
            c_Rotation[i] = R_initial;
            c_Translation[i] = P_initial;
            c_Quat[i] = c_Rotation[i];
            Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
            Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        }

        // triangulate point based on the solve pnp result
        //3、对第l帧----最后一帧之间的每一帧，都与最后一帧进行三角化 frame_num-1 表示当前帧
        triangulateTwoFramesWithDepth(i, Pose[i], frame_num - 1, Pose[frame_num - 1], sfm_f);
    }
    //3: triangulate l-----l+1 l+2 ... frame_num -2
    for (int i = l + 1; i < frame_num - 1; i++)
        triangulateTwoFramesWithDepth(l, Pose[l], i, Pose[i], sfm_f);
    //4: solve pnp l-1; triangulate l-1 ----- l
    //             l-2              l-2 ----- l

    //4、PNP求解从第l-1到第0帧的每一帧与第l帧之间的变换矩阵，并进行三角化
    for (int i = l - 1; i >= 0; i--) {
        //solve pnp
        Matrix3d R_initial = c_Rotation[i + 1];
        Vector3d P_initial = c_Translation[i + 1];
        if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
            return false;
        c_Rotation[i] = R_initial;
        c_Translation[i] = P_initial;
        c_Quat[i] = c_Rotation[i];
        Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
        Pose[i].block<3, 1>(0, 3) = c_Translation[i];
        //triangulate
        triangulateTwoFramesWithDepth(i, Pose[i], l, Pose[l], sfm_f);
    }
    //5: triangulate all other points

    //5、三角化其他未恢复的特征点。
    //至此得到了滑动窗口中所有图像帧的位姿以及特征点的3d坐标
    for (int j = 0; j < feature_num; j++) {
        if (sfm_f[j].state == true)
            continue;
        if ((int) sfm_f[j].observation.size() >= 2) {
            Vector3d point0;
            Vector2d point1;
            int frame_0 = sfm_f[j].observation[0].first;
            if (sfm_f[j].observation_depth[0].second < 0.1
                || sfm_f[j].observation_depth[0].second > 10) //max and min measurement
                continue;
            point0 = Vector3d(sfm_f[j].observation[0].second.x() * sfm_f[j].observation_depth[0].second,
                              sfm_f[j].observation[0].second.y() * sfm_f[j].observation_depth[0].second,
                              sfm_f[j].observation_depth[0].second);
            int frame_1 = sfm_f[j].observation.back().first;
            point1 = sfm_f[j].observation.back().second;
            Vector3d point_3d;
            //triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);

            Matrix3d Pose0_R = Pose[frame_0].block<3, 3>(0, 0);
            Matrix3d Pose1_R = Pose[frame_1].block<3, 3>(0, 0);
            Vector3d Pose0_t = Pose[frame_0].block<3, 1>(0, 3);
            Vector3d Pose1_t = Pose[frame_1].block<3, 1>(0, 3);

            Vector2d residual;
            Vector3d point1_reprojected;
            //triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
            point_3d = Pose0_R.transpose() * point0 - Pose0_R.transpose() * Pose0_t;//point in world;
            point1_reprojected = Pose1_R * point_3d + Pose1_t;

            residual = point1 - Vector2d(point1_reprojected.x() / point1_reprojected.z(),
                                         point1_reprojected.y() / point1_reprojected.z());

            if (residual.norm() < 1.0 / 460) {//reprojection error
                sfm_f[j].state = true;
                sfm_f[j].position[0] = point_3d(0);
                sfm_f[j].position[1] = point_3d(1);
                sfm_f[j].position[2] = point_3d(2);
            }
            //cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
        }
    }

    /*
        for (int i = 0; i < frame_num; i++)
        {
            q[i] = c_Rotation[i].transpose();
            cout << "solvePnP  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
        }
        for (int i = 0; i < frame_num; i++)
        {
            Vector3d t_tmp;
            t_tmp = -1 * (q[i] * c_Translation[i]);
            cout << "solvePnP  t" << " i " << i <<"  " << t_tmp.x() <<"  "<< t_tmp.y() <<"  "<< t_tmp.z() << endl;
        }
    */
    //full BA

    //6、使用cares进行全局BA优化
    ceres::Problem problem;
    ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();
    //cout << " begin full BA " << endl;
    for (int i = 0; i < frame_num; i++) {
        //double array for ceres
        c_translation[i][0] = c_Translation[i].x();
        c_translation[i][1] = c_Translation[i].y();
        c_translation[i][2] = c_Translation[i].z();
        c_rotation[i][0] = c_Quat[i].w();
        c_rotation[i][1] = c_Quat[i].x();
        c_rotation[i][2] = c_Quat[i].y();
        c_rotation[i][3] = c_Quat[i].z();
        problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
        problem.AddParameterBlock(c_translation[i], 3);
        if (i == l) {
            //固定优化变量
            problem.SetParameterBlockConstant(c_rotation[i]);
        }
        if (i == l || i == frame_num - 1) {
            problem.SetParameterBlockConstant(c_translation[i]);
        }
    }

    for (int i = 0; i < feature_num; i++) {
        if (sfm_f[i].state != true)
            continue;
        for (int j = 0; j < int(sfm_f[i].observation.size()); j++) {
            int l = sfm_f[i].observation[j].first;
            ceres::CostFunction *cost_function = ReprojectionError3D::Create(
                sfm_f[i].observation[j].second.x(),
                sfm_f[i].observation[j].second.y());

            problem.AddResidualBlock(cost_function, nullptr, c_rotation[l], c_translation[l],
                                     sfm_f[i].position);
        }

    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.2;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.BriefReport() << "\n";
    if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03) {
        //cout << "vision only BA converge" << endl;
    }
    else {
        //cout << "vision only BA not converge " << endl;
        return false;
    }
    //这里得到的是第l帧坐标系到各帧的变换矩阵，应将其转变为各帧在第l帧坐标系上的位姿
    for (int i = 0; i < frame_num; i++) {
        q[i].w() = c_rotation[i][0];
        q[i].x() = c_rotation[i][1];
        q[i].y() = c_rotation[i][2];
        q[i].z() = c_rotation[i][3];
        q[i] = q[i].inverse();
        //cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
    }
    for (int i = 0; i < frame_num; i++) {

        T[i] = -1 * (q[i] * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
        //cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<< T[i](2) << endl;
    }
    for (int i = 0; i < (int) sfm_f.size(); i++) {
        if (sfm_f[i].state)
            sfm_tracked_points[sfm_f[i].id] =
                Vector3d(sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2]);
    }
    return true;

}

