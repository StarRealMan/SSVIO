#ifndef __G20_OPTIM_H__
#define __G20_OPTIM_H__

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

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

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
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4],
            update[5];
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

class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly(const Eigen::Vector3d &pointworld):_pointworld(pointworld){}

    virtual void computeError() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        Eigen::Vector3d pointcam = T * _pointworld;
        _error = _measurement - pointcam;
    }

    virtual void linearizeOplus() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        Eigen::Vector3d pos_cam = T * _pointworld;
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18);
        double Zinv2 = Zinv * Zinv;
        //_jacobianOplusXi << ;
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
    Eigen::Vector3d _pointworld;
};


#endif