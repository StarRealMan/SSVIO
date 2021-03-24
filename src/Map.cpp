#include "Map.h"

Map::Map(Config::Ptr config)
{
    _final_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    _final_cloud->width = 0;
    _final_cloud->height = 1;
    _final_cloud->points.resize(_final_cloud->width * _final_cloud->height);
    _VoxelSize = config->GetParam<float>("VoxelSize");
}

Map::~Map()
{

}

void Map::Set2TrajVec(Eigen::Vector3f traj)
{
    std::lock_guard<std::mutex> lck(_traj_vec_mtx);
    _traj_vec.push_back(traj);
}

std::vector<Eigen::Vector3f>& Map::GetTrajVec()
{
    std::lock_guard<std::mutex> lck(_traj_vec_mtx);
    return _traj_vec;
}

void Map::Set2KeyFrameVec(Frame::Ptr key_frame)
{
    std::lock_guard<std::mutex> lck(_key_frame_vec_mtx);
    _key_frame_vec.push_back(key_frame);
}

Frame::Ptr Map::GetKeyFrames(int key_frame_id)
{
    std::lock_guard<std::mutex> lck(_key_frame_mtx);
    int key_frame_vec_size = _key_frame_vec.size();

    if(key_frame_id < 0)
    {
        return _key_frame_vec[key_frame_vec_size + key_frame_id];
    }
    else
    {
        return _key_frame_vec[key_frame_id];
    }
}

std::vector<Frame::Ptr>& Map::GetKeyFramesVec()
{
    std::lock_guard<std::mutex> lck(_key_frame_vec_mtx);
    return _key_frame_vec;
}

int Map::GetKeyFrameNum()
{
    std::lock_guard<std::mutex> lck(_key_frame_vec_mtx);
    return _key_frame_vec.size();
}

MapPoint::Ptr Map::GetMapPoint(int map_point_id)
{
    std::lock_guard<std::mutex> lck(_map_point_mtx);
    return _map_point_vec[map_point_id];
}

int Map::GetMapPointNum()
{
    return _map_point_vec.size();
}

void Map::TrackMapPoints(std::vector<cv::DMatch> &last_match_vec, std::vector<cv::DMatch> &this_match_vec)
{
    std::vector<cv::DMatch> temp_last_match_vec;

    for(size_t i = 0; i < last_match_vec.size(); i++)
    {
        for(int j = 0; j < this_match_vec.size(); j++)
        {
            if(last_match_vec[i].queryIdx == this_match_vec[j].trainIdx)
            {
                cv::DMatch temp_match;

                temp_match.trainIdx = last_match_vec[i].trainIdx;
                temp_match.queryIdx = this_match_vec[j].queryIdx;

                temp_last_match_vec.push_back(temp_match);
                break;
            }
        }
    }

    last_match_vec.clear();
    last_match_vec = temp_last_match_vec;
}

void Map::ManageMapPoints(Frame::Ptr key_frame, std::vector<cv::DMatch>& last_match_vec)
{
    Eigen::Matrix4f world_trans = key_frame->GetAbsPose().inverse();
    for(size_t i = 0; i < key_frame->GetKeyPoints().size(); i++)
    {
        int match_num = InMatchVec(i, last_match_vec);
        if(match_num == -1)
        {
            cv::Point3f cv_point_pose = key_frame->Get3DPoint(i);
            Eigen::Vector3f eigen_point_pose(cv_point_pose.x, cv_point_pose.y, cv_point_pose.z);
            MapPoint::Ptr map_point = std::make_shared<MapPoint>(world_trans.block<3,3>(0,0)*eigen_point_pose+
                                                                 world_trans.block<3,1>(0,3));
            map_point->SetObserve(key_frame->GetID(), i);
            _map_point_vec.push_back(map_point);
            key_frame->SetObserve(map_point->GetID(), i);
        }
        else
        {
            MapPoint::Ptr map_point = _map_point_vec[GetKeyFrames(-1)->GetMapPointID(last_match_vec[match_num].queryIdx)];

            map_point->SetObserve(key_frame->GetID(), i);
            key_frame->SetObserve(map_point->GetID(), i);
        }
    }
}

int Map::InMatchVec(int i, std::vector<cv::DMatch>& last_match_vec)
{
    for(int j = 0; j < last_match_vec.size(); j++)
    {
        if(last_match_vec[j].queryIdx == i)
        {
            return j;
        }
    }

    return -1;
}

void Map::MapPointCloudFusion()
{
    for(size_t i = 0; i < GetKeyFrameNum(); i++)
    {
        Frame::Ptr this_frame = GetKeyFrames(i);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr this_cloud =  boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        this_cloud = this_frame->GetRGBCloud();

        Eigen::Matrix4f this_trans = this_frame->GetAbsPose().inverse();
        Eigen::Matrix3f rotation = this_trans.block<3,3>(0,0);
        Eigen::Vector3f translation = this_trans.block<3,1>(0,3);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(rotation);
        transform.translate(translation);

        pcl::transformPointCloud(*this_cloud, *trans_cloud, transform);

        *_final_cloud += *trans_cloud;

        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(_final_cloud);
        sor.setLeafSize(_VoxelSize, _VoxelSize, _VoxelSize);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =  boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        sor.filter(*cloud_filtered);

        *_final_cloud = *cloud_filtered;
    }

    pcl::PCDWriter writer;
    writer.write("../savings/map.pcd", *_final_cloud);
    std::cout << "Final Point Cloud Map Has " << _final_cloud->size() << " Points!" << std::endl; 
}

