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
    // do edge outlier check
    _optimizer.initializeOptimization();
    _optimizer.optimize(optim_round);
}

void OdomOptimizer::AddPose(Eigen::Matrix4f pose_val)
{
     _vertex_pose= new VertexPose();
    _optimze_val = SE3(pose_val);
    _vertex_pose->setEstimate(_optimze_val);
    _vertex_pose->setId(0);
    _optimizer.addVertex(_vertex_pose);
}

void OdomOptimizer::AddMeasure(cv::Point3f cv_refered_point, cv::Point3f cv_measured_point, int measure_id)
{
    Eigen::Matrix<float, 3, 1> refered_point;
    Eigen::Matrix<float, 3, 1> measured_point;

    refered_point << cv_refered_point.x, cv_refered_point.y, cv_refered_point.z;
    measured_point << cv_measured_point.x, cv_measured_point.y, cv_measured_point.z;

    EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(refered_point);
    edge->setId(measure_id);
    edge->setVertex(0, _vertex_pose);
    edge->setMeasurement(measured_point);
    edge->setInformation(Eigen::Matrix3d::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    _optimizer.addEdge(edge);
}

Eigen::Matrix4f OdomOptimizer::GetPose()
{
    return _optimze_val.matrix();
}



LocalOptimizer::LocalOptimizer()
{

}

LocalOptimizer::~LocalOptimizer()
{

}

void LocalOptimizer::DoOptimization(int optim_round)
{

}

void LocalOptimizer::AddPose(Eigen::Matrix4f pose_val, int pose_id)
{

}

void LocalOptimizer::AddPoint(cv::Point3f cv_map_point, int measure_id)
{

}

void LocalOptimizer::AddMeasure(cv::Point3f cv_measured_point, int measure_id)
{

}


