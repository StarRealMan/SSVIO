#include "Odometry.h"

Odometry::Odometry(XtionCamera::Ptr camera, Config::Ptr config)
{
    _camera = camera;
    _FeatureNum = config->GetParam<int>("FeatureNum");
    _ScaleFactor = config->GetParam<float>("ScaleFactor");
    _LevelNum = config->GetParam<int>("LevelNum");
    _IniThFAST = config->GetParam<float>("IniThFAST");
    _MinThFAST = config->GetParam<float>("MinThFAST");
    _orb_extractor = std::make_shared<ORBextractor>(_FeatureNum, _ScaleFactor, _LevelNum, _IniThFAST, _MinThFAST);
    _feature_match = std::make_shared<FMatch>("../vocabulary/orbvoc.dbow3", config);

    Frame::_InnerCx = config->GetParam<float>("InnerCx");
    Frame::_InnerCy = config->GetParam<float>("InnerCy");
    Frame::_InnerFx = config->GetParam<float>("InnerFx");
    Frame::_InnerFy = config->GetParam<float>("InnerFy");
    Frame::_InvInnerFx = 1/Frame::_InnerFx;
    Frame::_InvInnerFy = 1/Frame::_InnerFy;
    Frame::_DepthScale = 10.0/65535.0;

    _InnerK = (cv::Mat_<float>(3,3) << Frame::_InnerFx, 0, Frame::_InnerCx, 0, Frame::_InnerFy, Frame::_InnerCy, 0, 0, 1);

    _odometry_running.store(true);
    _odometry_thread = std::thread(std::bind(&Odometry::OdometryLoop,this));
}

Odometry::~Odometry()
{

}


Frame::Ptr Odometry::GetCurFrame()
{
    std::lock_guard<std::mutex> lck(_cur_frame_mtx);
    return _cur_frame;
}

Eigen::Matrix4f Odometry::OptimizeTransform()
{
    Eigen::Matrix4f transform;
    std::vector<cv::Point3f> key_frame_3d_point_set;
    std::vector<cv::Point2f> cur_2d_point_set;
    cv::Mat r_vec, t_vec;

    for(int i = 0; i < _bow_match.size(); i++)
    {
        cv::Point3f key_frame_point;
        cv::Point2f cur_point;
        cv::DMatch match = _bow_match[i];

        key_frame_point = _key_frame_vec.back()->Get3DPoint(match.trainIdx);
        if(key_frame_point.z != 0)
        {
            key_frame_3d_point_set.push_back(key_frame_point);
            cur_point.x = _cur_frame->GetKeyPoints()[match.queryIdx].pt.x;
            cur_point.y = _cur_frame->GetKeyPoints()[match.queryIdx].pt.y;
            cur_2d_point_set.push_back(cur_point);
        }
    }

    if(key_frame_3d_point_set.size() > 10)
    {
        cv::solvePnP(key_frame_3d_point_set,cur_2d_point_set, _InnerK, cv::Mat(), r_vec, t_vec);
        // cv::Mat R, t;
        // cv::Rodrigues(rvec, R);
        // t = tvec.clone();
        // cv::Mat -> Eigen
        // set_init
    }
    else
    {
        // init set to identity
    }

    // add vertex
    for(int i = 0; i < _bow_match.size(); i++)
    {
        cv::Point3f key_frame_point;
        cv::Point3f cur_point;
        cv::DMatch match = _bow_match[i];

        cur_point = _cur_frame->Get3DPoint(match.queryIdx);
        key_frame_point = _key_frame_vec.back()->Get3DPoint(match.trainIdx);
        if(key_frame_point.z == 0 || cur_point.z == 0)
        {
            continue;
        }
        else
        {
            // add edge
        }
        
        // do the g2o optimization
    }
    
    return transform;
}

void Odometry::OdometryLoop()
{
    Eigen::Matrix4f transform;
    Eigen::Matrix4f last_key_framme_pose;

    while(_odometry_running.load())
    {
        if(_camera->IsGrabRdy())
        {
            _camera->SetGrabRdyfalse();

            auto t1 = std::chrono::steady_clock::now();
            std::cout << "<=============================================================>" << std::endl;
            
            _cur_frame = std::make_shared<Frame>(_camera->GetRGBImage(), _camera->GetDImage(), _orb_extractor);
            if(!_init_rdy)
            {
                _key_frame_vec.push_back(_cur_frame);
                _cur_frame->SetKeyFrame();
                _cur_frame->SetAbsPose(Eigen::Matrix4f::Identity());
                last_key_framme_pose = Eigen::Matrix4f::Identity();
                _init_rdy = true;
            }
            else
            {
                _feature_match->MatchByBOW(_cur_frame->GetDescriptor(), _key_frame_vec.back()->GetDescriptor(), _bow_match);
                transform = OptimizeTransform();

                _cur_frame->CheckKeyFrame(transform);
                if(_cur_frame->IsKeyFrame())
                {
                    last_key_framme_pose = transform * last_key_framme_pose;
                    _cur_frame->SetAbsPose(last_key_framme_pose);
                    _key_frame_vec.push_back(_cur_frame);
                }
            }

            auto t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout << "Odometry Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
        }
    }
}


void Odometry::OdometryStop()
{
    _odometry_running.store(false);
    _odometry_thread.join();
}
