#include "Frame.h"

Frame::Frame(cv::Mat rgb_img, cv::Mat d_img, ORBextractor::Ptr _orb_extractor):_rgb_img(rgb_img), _d_img(d_img)
{
    _is_key_frame = false;
    cv::Mat gray_image;
    cv::cvtColor( _rgb_img, gray_image, CV_RGB2GRAY);
    _orb_extractor->Extract(gray_image, cv::Mat(), _key_point_vec, _descriptor);
}

Frame::~Frame()
{
    
}

void Frame::CheckKeyFrame(Eigen::Matrix4f transform, int good_point_num, int frames_between)
{
    if(frames_between > _MaxFramesBetween)
    {
        SetKeyFrame();
    }
    else if(frames_between < _MinFramesBetween)
    {
        return;
    }
    else
    {
        if(good_point_num > _MaxGoodPointThres)
        {
            return;
        }
        else
        {
            SetKeyFrame();
        }
    }
}

bool Frame::IsKeyFrame()
{
    std::lock_guard<std::mutex> lck(_is_key_frame_mtx);
    return _is_key_frame;
}

void Frame::SetKeyFrame()
{
    std::lock_guard<std::mutex> lck(_is_key_frame_mtx);
    _is_key_frame = true;
    _key_frame_id = _key_frame_point_num;
    _key_frame_point_num++;
}

cv::Mat Frame::GetDImage()
{
    std::lock_guard<std::mutex> lck(_d_image_mtx);
    return _d_img;
}

cv::Mat Frame::GetRGBImage()
{
    std::lock_guard<std::mutex> lck(_rgb_image_mtx);
    return _rgb_img;
}

std::vector<cv::KeyPoint> Frame::GetKeyPoints()
{
    std::lock_guard<std::mutex> lck(_key_points_mtx);
    return _key_point_vec;
}

cv::Mat Frame::GetDescriptor()
{
    std::lock_guard<std::mutex> lck(_descriptor_mtx);
    return _descriptor;
}

void Frame::SetAbsPose(Eigen::Matrix4f pose)
{
    std::lock_guard<std::mutex> lck(_descriptor_mtx);
    if(_is_key_frame)
        _rel_abs_pos = pose;
}

cv::Point3f Frame::Get3DPoint(int index)
{
    cv::Point3f temp_3d_point;
    float pos_x, pos_y;

    pos_x = _key_point_vec[index].pt.x;
    pos_y = _key_point_vec[index].pt.y;
    
    float depth = _d_img.at<ushort>(pos_y, pos_x)*_DepthScale;

    temp_3d_point.x = (pos_x - _InnerCx)*depth*_InvInnerFx;
    temp_3d_point.y = (pos_y - _InnerCy)*depth*_InvInnerFy;
    temp_3d_point.z = depth;

    return temp_3d_point;
}

int Frame::GetID()
{
    if(_is_key_frame)
    {
        std::lock_guard<std::mutex> lck(_id_mtx);
        return _key_frame_id;
    }
    else
    {
        return 0;
    }
}

void Frame::AddObserveIdx(int img_point_idx, int map_point_id)
{
    std::pair<int, int> idx_pair;
    idx_pair.first = img_point_idx;
    idx_pair.second = map_point_id;
    _idx_pair_vec.push_back(idx_pair);
}

void Frame::GetObserveIdx(std::vector<std::pair<int, int>> &idx_pair_vec)
{
    idx_pair_vec = _idx_pair_vec;
}
