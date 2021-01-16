#include "Map.h"

Map::Map(Xtion_Camera::Ptr camera, Config::Ptr config)
{
    _mapcam = camera;
    _voxel_size = config->GetParam<float>("Voxel_size");
    _mapcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    _mapcloud->width = 0;
    _mapcloud->height = 1;
    _mapcloud->points.resize(_mapcloud->width * _mapcloud->height);
    _current_pose = Eigen::Matrix4f::Identity();

    _height = 240;
    _width = 320;
    _inner_cx = 160.5912;
    _inner_cy = 120.4792;
    _inner_fx = 253.0589;
    _inner_fy = 254.1649;
    _inv_inner_fx = 1/_inner_fx;
    _inv_inner_fy = 1/_inner_fy;

    _new_pos_set = false;
    _maprunning.store(true);
    _mapthread = std::thread(std::bind(&Map::MapLoop,this));
}

Map::~Map()
{

}

void Map::setPose(const Eigen::Matrix4f& pose)
{
    std::lock_guard<std::mutex> lck(_pose_mtx);
    _current_pose = pose;
    _new_pos_set = true;
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


void Map::UpdateMap(void)
{
    std::lock_guard<std::mutex> lck(_pose_mtx);
    std::cout << "Current Map has " << _mapcloud->width << " Points!" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // do new_cloud = _current_pose * PointCam
    PointcloudTransform(new_cloud, _current_pose);

    *_mapcloud += *new_cloud;

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(_mapcloud);
    sor.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
    sor.filter(*cloud_filtered);

    *_mapcloud = *cloud_filtered;
}


void Map::MapLoop(void)
{
    while(_maprunning.load())
    {
        if(_new_pos_set == true)
        {
            UpdateMap();
            _new_pos_set = false;
        }
    }
}

void Map::MapStop()
{
    _maprunning.store(false);
    _mapthread.join();
}