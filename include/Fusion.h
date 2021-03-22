#ifndef __FUSION_H__
#define __FUSION_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

class Fusion
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Fusion> Ptr;

    Fusion();
    ~Fusion();

private:

};

#endif