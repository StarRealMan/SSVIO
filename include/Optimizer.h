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

#include "Config.h"

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

class VertexPoint : public g2o::BaseVertex<3, Eigen::Vector3f>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override
    {
        _estimate = Eigen::Vector3f::Zero();
    }

    virtual void oplusImpl(const double *update) override
    {
        Eigen::Matrix<float, 3, 1> update_eigen;
        update_eigen << update[0], update[1], update[2];
        _estimate = update_eigen + _estimate;
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

class EdgeICPPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3f, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeICPPoseOnly(const Eigen::Vector3f &key_frame_point):BaseUnaryEdge(), _ref_frame_point(key_frame_point){}

    virtual void computeError() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        _error = _measurement.cast<double>() - T.cast<double>() * _ref_frame_point.cast<double>();
    }

    virtual void linearizeOplus() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        Eigen::Vector3f pos_cam = T * _ref_frame_point;
        _jacobianOplusXi.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3,3>(0,3) =  Sophus::SO3d::hat(pos_cam.cast<double>());
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
    Eigen::Vector3f _ref_frame_point;
};

class EdgeIMUPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Matrix3f, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeIMUPoseOnly(const Eigen::Matrix4f &key_frame_pose):BaseUnaryEdge(), _ref_frame_pose(key_frame_pose){}

    virtual void computeError() override
    {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T_curr = v->estimate();
        SO3 T_m(_measurement);
        SO3 T_key(_ref_frame_pose.block<3,3>(0,0));
        _error = ((T_m.cast<double>() * T_key.cast<double>()) * T_curr.so3().cast<double>().inverse()).log();
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
    Eigen::Matrix4f _ref_frame_pose;
};

class EdgeICPPosePoint : public g2o::BaseBinaryEdge<3, Eigen::Vector3f, VertexPose, VertexPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeICPPosePoint():BaseBinaryEdge(){}

    virtual void computeError() override
    {
        const VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
        const VertexPoint *point = static_cast<VertexPoint *>(_vertices[1]);
        
        SE3 T = pose->estimate();
        Eigen::Vector3f P = point->estimate();
        _error = _measurement.cast<double>() - T.cast<double>() * P.cast<double>();
    }

    virtual void linearizeOplus() override
    {
        const VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
        const VertexPoint *point = static_cast<VertexPoint *>(_vertices[1]);
        SE3 T = pose->estimate();
        Eigen::Vector3f P = point->estimate();
        Eigen::Vector3f pos_cam = T * P;
        _jacobianOplusXi.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
        _jacobianOplusXi.block<3,3>(0,3) =  Sophus::SO3d::hat(pos_cam.cast<double>());

        _jacobianOplusXj = T.matrix().block<3,3>(0,0).cast<double>();
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





class OdomOptimizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<OdomOptimizer> Ptr;

    OdomOptimizer();
    ~OdomOptimizer();

    void DoOptimization(int optim_round);
    void AddPose(Eigen::Matrix4f pose_val);
    void AddCVMeasure(cv::Point3f cv_refered_point, cv::Point3f cv_measured_point, int measure_id);
    void AddIMUMeasure(Eigen::Matrix4f ref_frame_pose, Eigen::Matrix3f imu_measured_pose, int measure_id);
    Eigen::Matrix4f GetPose();

private:
    static float _IMUGain;
    g2o::SparseOptimizer _optimizer;
    VertexPose *_vertex_pose;
    SE3 _optimze_val;
};

class LocalOptimizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<LocalOptimizer> Ptr;

    LocalOptimizer();
    ~LocalOptimizer();

    void DoOptimization(int optim_round);
    void AddPose(Eigen::Matrix4f pose_val, int pose_id);
    void AddPoint(cv::Point3f cv_map_point, int measure_id);
    void AddMeasure(cv::Point3f cv_measured_point, int measure_id, int pose_id, int point_id);

private:
    g2o::SparseOptimizer _optimizer;
    std::vector<VertexPose*> _vertex_pose_vec;
    std::vector<VertexPoint*> _vertex_point_vec;
    std::vector<Eigen::Vector3f> _optimze_point_val_vec;
    std::vector<SE3> _optimze_pose_val_vec;
};


#endif

