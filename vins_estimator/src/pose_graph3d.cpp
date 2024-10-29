//
// Created by ubuntu on 2020/5/30.
//

#include <iostream>
#include <fstream>
#include "glog/logging.h"

#include "types.h"
#include "read_g2o.h"
#include "factor/FrameParameterization.h"
#include "factor/SE3RelativtePoseFactor.h"

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");


void BuildOptimizationProblem(const VectorOfConstraints &constraints,
                              MapOfPoses *poses, ceres::Problem *problem)
{
    CHECK(poses != nullptr);
    CHECK(problem != nullptr);

    if (constraints.empty()) {
        LOG(INFO) << "No constraint, no problem to optimize.";
        return;
    }

    ceres::LossFunction *loss_function = nullptr;

    FrameParameterization *frameParameterization = new FrameParameterization();

    for (auto &constraint : constraints) {
        auto pose_i_iter = poses->find(constraint.host_id);
        CHECK(pose_i_iter != poses->end())
        << "Pose with ID: " << constraint.host_id << " not found";

        auto pose_j_iter = poses->find(constraint.target_id);
        CHECK(pose_j_iter != poses->end())
        << "Pose with ID: " << constraint.target_id << " not found";

        const Eigen::Matrix<double, 6, 6> sqrt_information =
            constraint.information.llt().matrixL();

        Sophus::SE3d T_i_j_estimate(constraint.T_h_t.rot(), constraint.T_h_t.pos());
        ceres::CostFunction *relpose_factor = new SE3RelativtePoseFactor(T_i_j_estimate, sqrt_information);

        problem->AddResidualBlock(relpose_factor, loss_function,
                                  pose_i_iter->second.data.data(),
                                  pose_j_iter->second.data.data());

        problem->SetParameterization(pose_i_iter->second.data.data(), frameParameterization);
        problem->SetParameterization(pose_j_iter->second.data.data(), frameParameterization);
    }

    auto pose_start_iter = poses->begin();
    CHECK(pose_start_iter != poses->end()) << "There are no poses.";

    problem->SetParameterBlockConstant(pose_start_iter->second.data.data());
    problem->SetParameterBlockConstant((*poses)[1].data.data());
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem *problem)
{
    CHECK(problem != nullptr);

    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << '\n';

    return summary.IsSolutionUsable();
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string &filename, const MapOfPoses &poses)
{
    std::fstream outfile;
    outfile.open(filename.c_str(), std::istream::out);
    if (!outfile) {
        LOG(ERROR) << "Error opening the file: " << filename;
        return false;
    }
    for (const auto &pair : poses) {
        outfile << pair.first << " " << pair.second.data[4] << " " << pair.second.data[5] << " " << pair.second.data[6]
                << " " << pair.second.data[0] << " " << pair.second.data[1] << " "
                << " " << pair.second.data[2] << " " << pair.second.data[3] << '\n';
    }
    return true;
}

int main(int argc, char **argv)
{
#if 1
    Pose3d pose1;
    LOG(INFO) << pose1.data[7];
    pose1.data.back() = 1;
    LOG(INFO) << pose1.getId();

    pose1.setId(100);
    LOG(INFO) << int64_t(pose1.data[7]);
    LOG(INFO) << pose1.getId();

    pose1.data.back() = 10;
    LOG(INFO) << pose1.getId();


    if (pose1.getType() == 'p') {
        LOG(INFO) << "this parameter is pose";
    }

    Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
    LOG(INFO) << "qx qy qz qw :" << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    pose1.setRot(q);


    Eigen::Quaterniond q2 = pose1.rot();
    LOG(INFO) << "qx qy qz qw :" << q2.x() << " " << q2.y() << " " << q2.z() << " " << q2.w();

    Eigen::Map<Eigen::Vector3d> t(pose1.data.data()+4);
    t<< 0., 1., 2.;

    LOG(INFO) << "tx ty tz:" << pose1.data[4] << " " << pose1.data[5]  << " " << pose1.data[6];
#endif
    google::InitGoogleLogging(argv[0]);
    GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

    MapOfPoses poses;
    VectorOfConstraints constraints;

    CHECK(ReadG2oFile(FLAGS_input, &poses, &constraints))
    << "Error reading the file: " << FLAGS_input;


    std::cout << "Number of poses: " << poses.size() << '\n';

    if (poses[0].getType() == 'p') {
        std::cout << "poses[0] type is pose" <<std::endl;
    }

    if (Trait::getType(poses[0].data.data() + 7) == 'p') {
        std::cout << "poses[0] type is pose"<< std::endl;
        std::cout << "pose type: "<< Trait::getType(poses[0].data.data() + 7) << std::endl;
    }
    std::cout << "Number of constraints: " << constraints.size() << '\n';

    CHECK(OutputPoses("poses_original.txt", poses))
    << "Error outputing to pose_original.txt";

    ceres::Problem problem;
    BuildOptimizationProblem(constraints, &poses, &problem);

    CHECK(SolveOptimizationProblem(&problem))
    << "The solve was not successful, exiting.";

    CHECK(OutputPoses("poses_optimized.txt", poses))
    << "Error outputting to poses_original.txt";

    return 0;
}