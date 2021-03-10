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
    _feature_match = std::make_shared<FeatureMatching>("../vocabulary/orbvoc.dbow3", config);
    _InnerK = (cv::Mat_<float>(3,3) << Frame::_InnerFx, 0, Frame::_InnerCx, 0, Frame::_InnerFy, Frame::_InnerCy, 0, 0, 1);
    _frames_between = 0;
    _init_rdy = false;

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
    std::vector<cv::Point2f> cur_frame_2d_point_set;
    std::vector<cv::Point3f> cur_frame_3d_point_set;
    cv::Mat r_vec, t_vec;
    
    _final_good_point_num = 0;

    for(unsigned int i = 0; i < _bow_match.size(); i++)
    {
        cv::Point3f key_frame_point;
        cv::Point2f cur_frame_point_2d;
        cv::DMatch match = _bow_match[i];

        key_frame_point = _key_frame_vec.back()->Get3DPoint(match.trainIdx);
        if(key_frame_point.z >= Frame::_DepthScale)
        {
            key_frame_3d_point_set.push_back(key_frame_point);
            cur_frame_point_2d.x = _cur_frame->GetKeyPoints()[match.queryIdx].pt.x;
            cur_frame_point_2d.y = _cur_frame->GetKeyPoints()[match.queryIdx].pt.y;
            cur_frame_2d_point_set.push_back(cur_frame_point_2d);

            cv::Point3f cur_frame_point_3d;
            cur_frame_point_3d = _cur_frame->Get3DPoint(match.queryIdx);
            cur_frame_3d_point_set.push_back(cur_frame_point_3d);
        }
    }

    if(key_frame_3d_point_set.size() > 12)
    {
        cv::solvePnP(key_frame_3d_point_set,cur_frame_2d_point_set, _InnerK, cv::Mat(), r_vec, t_vec);
        cv::Mat R, t;

        cv::Rodrigues(r_vec, R);
        t = t_vec.clone();

        Eigen::Matrix3f R_eigen;
        Eigen::Vector3f t_eigen;
        cv::cv2eigen(R,R_eigen);
        cv::cv2eigen(t,t_eigen);

        transform.block<3,3>(0,0) = R_eigen;
        transform.block<3,1>(0,3) = t_eigen;
        transform.block<1,3>(3,0) = Eigen::RowVector3f::Zero();
        transform(3,3) = 1;
    }
    else
    {
        transform = Eigen::Matrix4f::Identity();
    }

    Optimizer::Ptr optimizer(new Optimizer);
    optimizer->AddPose(transform);
    for(unsigned int i = 0; i < key_frame_3d_point_set.size(); i++)
    {
        cv::Point3f key_frame_point;
        cv::Point3f cur_frame_point;

        cur_frame_point = cur_frame_3d_point_set[i];
        key_frame_point = key_frame_3d_point_set[i];

        if(cur_frame_point.z >= Frame::_DepthScale)
        {
            optimizer->AddMeasure(key_frame_point, cur_frame_point, i);
            _final_good_point_num++;
        }
    }

    if(_final_good_point_num > 10)
    {
        std::cout << "Start Optimize" << std::endl;
        optimizer->DoOptimization(10);
    }
    
    transform = optimizer->GetPose();
    return transform;
}

void Odometry::OdometryLoop()
{
    Eigen::Matrix4f transform;
    Eigen::Matrix4f last_key_frame_pose;
    cv::Mat cur_frame_desp;
    cv::Mat last_key_frame_desp;

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
                last_key_frame_pose = Eigen::Matrix4f::Identity();
                last_key_frame_desp = _cur_frame->GetDescriptor();
                _init_rdy = true;
            }
            else
            {
                cur_frame_desp = _cur_frame->GetDescriptor();
                std::cout << "Found " << cur_frame_desp.rows << " Key Points" << std::endl;

                _feature_match->MatchByDBoW(cur_frame_desp, last_key_frame_desp, _bow_match);
                std::cout << "Found " << _bow_match.size() << " Matches" << std::endl;

                transform = OptimizeTransform();
                std::cout << transform << std::endl;

                _cur_frame->CheckKeyFrame(transform, _final_good_point_num, _frames_between);
                if(_cur_frame->IsKeyFrame())
                {
                    last_key_frame_pose = transform * last_key_frame_pose;
                    _cur_frame->SetAbsPose(last_key_frame_pose);
                    _key_frame_vec.push_back(_cur_frame);
                    last_key_frame_desp = cur_frame_desp;
                    _frames_between = 0;
                    std::cout << "Add a Key Frame, Now Num: " << _key_frame_vec.size() << std::endl;
                }
                else
                {
                    _frames_between++;
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
