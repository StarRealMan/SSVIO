#include "Map.h"

Map::Map(Xtion_Camera::Ptr camera, Config::Ptr config)
{
    _mapcam = camera;
    _voxel_size = config->GetParam<float>("Voxel_size");
    _mapcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    _mapcloud->width = 0;
    _mapcloud->height = 1;
    _mapcloud->points.resize(_mapcloud->width * _mapcloud->height);

    _height = 240;
    _width = 320;
    _inner_cx = 160.5912;
    _inner_cy = 120.4792;
    _inner_fx = 253.0589;
    _inner_fy = 254.1649;
    _inv_inner_fx = 1/_inner_fx;
    _inv_inner_fy = 1/_inner_fy;
}

Map::~Map()
{

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map::getMapPointCloud()
{
    std::lock_guard<std::mutex> lck(_mappointcloud_mtx);
    return _mapcloud;
}

void Map::PointcloudTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud,const Eigen::Matrix4f& trans)
{
    Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
    Eigen::Vector3f translation = trans.block<3,1>(0,3);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(rotation);
    transform.translate(translation);

    pcl::transformPointCloud(*(_mapcam->getRGBCloud()), *new_cloud, transform);
}


void Map::UpdateMap(const Eigen::Matrix4f& pose)
{
    std::cout << " current Map has " << _mapcloud->width << " Points!" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    PointcloudTransform(new_cloud, pose);
    std::cout << " New cloud has " << new_cloud->width << " Points!" << std::endl;

    *_mapcloud += *new_cloud;

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(_mapcloud);
    sor.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
    sor.filter(*cloud_filtered);

    *_mapcloud = *cloud_filtered;
}


    