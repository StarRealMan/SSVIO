#ifndef __OPTIMIZER_H__
#define __OPTIMIZER_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

typedef Sophus::SE3f SE3;
typedef Sophus::SO3f SO3;

class VertexPose : public g2o::BaseVertex<6, SE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override
    {
        _estimate = SE3();
    }

    virtual void oplusImpl(const double *update) override
    {
        Eigen::Matrix<float, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4],update[5];
        _estimate = SE3::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &in) override
    {
        return true;
    }

    virtual bool write(std::ostream &out) const override
    {
        return true;
    }
};

class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3f, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly(const Eigen::Vector3f &key_frame_point):BaseUnaryEdge(), _key_frame_point(key_frame_point){}

    virtual void computeError() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        _error = T.cast<double>() * _key_frame_point.cast<double>() - _measurement.cast<double>();
    }

    virtual void linearizeOplus() override
    {
        // ???
    }

    virtual bool read(std::istream &in) override
    {
        return true;
    }

    virtual bool write(std::ostream &out) const override
    {
        return true;
    }

private:
    Eigen::Vector3f _key_frame_point;
};

class Optimizer
{
public:
    typedef std::shared_ptr<Optimizer> Ptr;

    Optimizer();
    ~Optimizer();

    void DoOptimization(int optim_round);
    void AddPose(Eigen::Matrix4f pose_val);
    void AddMeasure(cv::Point3f cv_refered_point, cv::Point3f cv_measured_point, int measure_id);
    Eigen::Matrix4f GetPose();

private:
    g2o::SparseOptimizer _optimizer;
    VertexPose *_vertex_pose;
    SE3 _optimze_val;
};

#endif

