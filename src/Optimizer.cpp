#include "Optimizer.h"

OdomOptimizer::OdomOptimizer()
{
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg
                     (g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    _optimizer.setAlgorithm(solver);
}

OdomOptimizer::~OdomOptimizer()
{

}

void OdomOptimizer::DoOptimization(int optim_round)
{
    for(int iteration = 0; iteration < 4; ++iteration)
    {
        // _optimizer.setVerbose(true);
        _vertex_pose->setEstimate(_optimze_val);
        _optimizer.initializeOptimization();
        _optimizer.optimize(optim_round);

        for(int i = 0; i < _edge_n_lier.size(); ++i)
        {
            auto e = _edge_n_lier[i];
            if(e.second)
            {
                e.first->computeError();
            }
            if(e.first->chi2() > _Chi2Thresh)
            {
                e.second = true;
                e.first->setLevel(1);
            }
            else
            {
                e.second = false;
                e.first->setLevel(0);
            };

            if(iteration == 2)
            {
                e.first->setRobustKernel(nullptr);
            }
        }
    }

}

void OdomOptimizer::AddPose(Eigen::Matrix4f pose_val)
{
    _vertex_pose = new VertexPose();
    _optimze_val = SE3(pose_val);
    _vertex_pose->setEstimate(_optimze_val);
    _vertex_pose->setId(0);
    _optimizer.addVertex(_vertex_pose);
}

void OdomOptimizer::AddCVMeasure(cv::Point3f cv_refered_point, cv::Point3f cv_measured_point, int measure_id)
{
    Eigen::Matrix<float, 3, 1> refered_point;
    Eigen::Matrix<float, 3, 1> measured_point;

    refered_point << cv_refered_point.x, cv_refered_point.y, cv_refered_point.z;
    measured_point << cv_measured_point.x, cv_measured_point.y, cv_measured_point.z;

    EdgeICPPoseOnly *edge = new EdgeICPPoseOnly(refered_point);
    edge->setId(measure_id);
    edge->setVertex(0, _vertex_pose);
    edge->setMeasurement(measured_point);
    edge->setInformation(Eigen::Matrix3d::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    _edge_n_lier[measure_id] = std::pair<EdgeICPPoseOnly*, bool>(edge, false);
    _optimizer.addEdge(edge);
}

void OdomOptimizer::AddIMUMeasure(Eigen::Matrix4f ref_frame_pose, Eigen::Matrix3f imu_measured_pose, int measure_id)
{
    Eigen::Matrix<float, 3, 1> refered_point;
    Eigen::Matrix<float, 3, 1> measured_point;

    EdgeIMUPoseOnly *edge = new EdgeIMUPoseOnly(ref_frame_pose);
    edge->setId(measure_id);
    edge->setVertex(0, _vertex_pose);
    edge->setMeasurement(imu_measured_pose);
    edge->setInformation(Eigen::Matrix3d::Identity() * _IMUGain);
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    _optimizer.addEdge(edge);
}

Eigen::Matrix4f OdomOptimizer::GetPose()
{
    return _optimze_val.matrix();
}



LocalOptimizer::LocalOptimizer()
{
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg
                     (g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    _optimizer.setAlgorithm(solver);
}

LocalOptimizer::~LocalOptimizer()
{

}

void LocalOptimizer::DoOptimization(int optim_round)
{
    // do edge outlier check
    _optimizer.initializeOptimization();
    _optimizer.optimize(optim_round);
}

void LocalOptimizer::AddPose(Eigen::Matrix4f pose_val, int pose_id)
{
    VertexPose *vertex_pose = new VertexPose();
    _optimze_pose_val_map[pose_id] = SE3(pose_val);
    vertex_pose->setEstimate(_optimze_pose_val_map[pose_id]);
    vertex_pose->setId(pose_id);
    _optimizer.addVertex(vertex_pose);
    _vertex_pose_map[pose_id] = vertex_pose;
}

void LocalOptimizer::AddPoint(Eigen::Vector3f map_point, int point_id)
{
    VertexPoint *vertex_point = new VertexPoint();
    _optimze_point_val_map[point_id] = map_point;
    vertex_point->setEstimate(_optimze_point_val_map[point_id]);
    vertex_point->setId(point_id);
    _optimizer.addVertex(vertex_point);
    _vertex_point_map[point_id] = vertex_point;
}

void LocalOptimizer::AddMeasure(cv::Point3f cv_measured_point, int measure_id, int pose_id, int point_id)
{
    Eigen::Matrix<float, 3, 1> measured_point;

    measured_point << cv_measured_point.x, cv_measured_point.y, cv_measured_point.z;

    EdgeICPPosePoint *edge = new EdgeICPPosePoint();
    edge->setId(measure_id);
    edge->setVertex(0, _vertex_pose_map[pose_id]);
    edge->setVertex(1, _vertex_point_map[point_id]);
    edge->setMeasurement(measured_point);
    edge->setInformation(Eigen::Matrix3d::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    _optimizer.addEdge(edge);
}


