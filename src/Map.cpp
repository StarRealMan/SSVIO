#include "Map.h"

Map::Map(Xtion_Camera::Ptr camera)
{
    _mapcam = camera;
    _mapcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    _mapcloud->width = 0;
    _mapcloud->height = 1;
    _mapcloud->points.resize (_mapcloud->width * _mapcloud->height);

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

void Map::UpdateMap(Eigen::Matrix4f pose)
{
    int origin_poin_num = _mapcloud->width;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered2(new pcl::PCLPointCloud2 ());

    _mapcloud->width = _mapcloud->width + _height *  _width;
    _mapcloud->height = 1;
    _mapcloud->points.resize(_mapcloud->width * _mapcloud->height);

    for(ushort u = 0; u < _width; u++)
    {
        for(ushort v = 0; v < _height; v++)
        {
            int i = u * _height + v;
            _mapcloud->points[i+origin_poin_num] = _mapcam->getRGB3DPoint(u,v,pose);
        }
    }

    pcl::toPCLPointCloud2(*_mapcloud,*cloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud2);
    sor.setLeafSize (1.0f, 1.0f, 1.0f);
    sor.filter (*cloud_filtered2);
    pcl::fromPCLPointCloud2(*cloud_filtered2,*cloud_filtered);
    *_mapcloud = *cloud_filtered;

}



    