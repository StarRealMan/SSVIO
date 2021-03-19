#include "Odometry.h"

Odometry::Odometry(XtionCamera::Ptr camera, IMU::Ptr imu, Map::Ptr map, Config::Ptr config)
{
    _camera = camera;
    _map = map;
    _imu = imu;
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

Eigen::Matrix4f Odometry::OptimizeTransform(Eigen::Matrix3f imu_trans_measure, Eigen::Vector3f imu_trans_t_measure)
{
    Eigen::Matrix4f transform;
    std::vector<cv::Point3f> last_frame_3d_point_set;
    std::vector<cv::Point2f> cur_frame_2d_point_set;
    std::vector<cv::Point3f> cur_frame_3d_point_set;
    std::vector<cv::DMatch> pre_good_match;
    cv::Mat r_vec, t_vec;
    int edge_num = 0;

    for(unsigned int i = 0; i < _bow_match.size(); i++)
    {
        cv::Point3f last_frame_point;
        cv::Point2f cur_frame_point_2d;
        cv::DMatch match = _bow_match[i];

        last_frame_point = _last_frame->Get3DPoint(match.trainIdx);
        if(last_frame_point.z >= Frame::_DepthScale)
        {
            last_frame_3d_point_set.push_back(last_frame_point);
            cur_frame_point_2d.x = _cur_frame->GetKeyPoints()[match.queryIdx].pt.x;
            cur_frame_point_2d.y = _cur_frame->GetKeyPoints()[match.queryIdx].pt.y;
            cur_frame_2d_point_set.push_back(cur_frame_point_2d);

            cv::Point3f cur_frame_point_3d;
            cur_frame_point_3d = _cur_frame->Get3DPoint(match.queryIdx);
            cur_frame_3d_point_set.push_back(cur_frame_point_3d);

            pre_good_match.push_back(match);
        }
    }

    if(last_frame_3d_point_set.size() > 30)
    {
        cv::Mat pnp_inliers;
        cv::solvePnPRansac(last_frame_3d_point_set,cur_frame_2d_point_set, _InnerK, cv::Mat(),
                           r_vec, t_vec, false, 100, (8.0F), 0.99, pnp_inliers);
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

    OdomOptimizer::Ptr optimizer(new OdomOptimizer);
    optimizer->AddPose(transform);

    for(unsigned int i = 0; i < pre_good_match.size(); i++)
    {
        cv::Point3f last_frame_point;
        cv::Point3f cur_frame_point;

        cur_frame_point = cur_frame_3d_point_set[i];
        last_frame_point = last_frame_3d_point_set[i];

        if(cur_frame_point.z >= Frame::_DepthScale)
        {
            optimizer->AddCVMeasure(last_frame_point, cur_frame_point, edge_num);
            _final_good_match.push_back(pre_good_match[i]);
            edge_num++;
        }
    }

    optimizer->AddIMUMeasure(_last_frame->GetAbsPose(), imu_trans_measure, edge_num);
    
    if(_final_good_match.size() > 20)
    {
        std::cout << "Start Optimize" << std::endl;
        optimizer->DoOptimization(20);
        transform = optimizer->GetPose();
    }
    else
    {
        std::cout << "Not Enough Good Points" << std::endl;
        transform.block<3,3>(0,0) = imu_trans_measure;
        // transform.block<3,1>(0,3) = imu_trans_t_measure;
        transform.block<3,1>(0,3) = Eigen::Vector3f::Zero();
        transform.block<1,3>(3,0) = Eigen::RowVector3f::Zero();
        transform(3,3) = 1;
    }
    
    return transform;
}

void Odometry::OdometryLoop()
{
    Eigen::Matrix4f transform;
    std::vector<cv::DMatch> last_match_vec;    

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
                _map->ManageMapPoints(_cur_frame, last_match_vec);
                _map->Set2KeyFrameVec(_cur_frame);
                _cur_frame->SetKeyFrame();
                _cur_frame->SetAbsPose(Eigen::Matrix4f::Identity());
                std::cout << "Add a Key Frame, Now Num: " << _map->GetKeyFrameNum() << std::endl;

                Eigen::Matrix3f imu_rotate_measure;
                Eigen::Vector3f imu_transit_measure;
                _imu->GetIMURotateData(imu_rotate_measure);
                _imu->GetIMUTransitData(imu_transit_measure);

                _init_rdy = true;
            }
            else
            {
                cv::Mat cur_frame_desp;
                cv::Mat last_frame_desp;
                cur_frame_desp = _cur_frame->GetDescriptor();
                last_frame_desp = _last_frame->GetDescriptor();
                _feature_match->MatchByDBoW(cur_frame_desp, last_frame_desp, _bow_match);

                std::cout << "Found " << cur_frame_desp.rows << " Key Points" << std::endl;
                std::cout << "Found " << _bow_match.size() << " Matches" << std::endl;

                Eigen::Matrix3f imu_rotate_measure;
                Eigen::Vector3f imu_transit_measure;
                _imu->GetIMURotateData(imu_rotate_measure);
                _imu->GetIMUTransitData(imu_transit_measure);
                
                transform = OptimizeTransform(imu_rotate_measure, imu_transit_measure);
                std::cout << transform << std::endl;

                _cur_frame->SetAbsPose(transform * _last_frame->GetAbsPose());
                
                if(_last_frame->IsKeyFrame())
                {
                    last_match_vec.clear();
                    last_match_vec = _bow_match;
                }
                else
                {
                    _map->TrackMapPoints(last_match_vec, _bow_match);
                }

                _cur_frame->CheckKeyFrame(transform, _final_good_match.size(), _frames_between);
                if(_cur_frame->IsKeyFrame())
                {
                    _map->ManageMapPoints(_cur_frame, last_match_vec);
                    _map->Set2KeyFrameVec(_cur_frame);
                    std::cout << "Add a Key Frame, Now Key Frame Num: " << _map->GetKeyFrameNum() << std::endl;
                    std::cout << "Now the Map Point Num: " << _map->GetMapPointNum() << std::endl;

                    _frames_between = 0;
                }
                else
                {
                    _frames_between++;
                }
                _final_good_match.clear();
            }

            _last_frame = _cur_frame;

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
