#include "Visual_Odometry.h"

VO::VO(Xtion_Camera::Ptr camera, Config::Ptr config)
{
    _vocam = camera;
    _featurepoint_coe = config->GetParam<float>("Featurepoint_coe");
    _featurepoint_max = config->GetParam<float>("Featurepoint_max");
    _goodmatch_thresh = config->GetParam<int>("Goodmatch_thresh");
    _chi2_thresh = config->GetParam<float>("Chi2_thresh");
    _optim_round = config->GetParam<int>("Optim_round");
    _map = std::make_shared<Map>(_vocam,config);
    _poses.push_back(Eigen::Matrix4f::Identity());
    _bfmatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    _vorunning.store(true);
    _vothread = std::thread(std::bind(&VO::VOLoop,this));
}

VO::~VO()
{
    pcl::PCDWriter Pclwriter;
    Pclwriter.write("../savings/map.pcd",*(_map->getMapPointCloud()));
}

Eigen::Matrix4f VO::Feature_Optimize()
{
    Eigen::Matrix4f pose = _lastframe->getPose();
    SE3 pose_estimate;
    int goodmatchpoint_size;
    _goodmatchepoints.clear();

    std::vector<cv::KeyPoint> featurepoints;
    std::vector<cv::KeyPoint> lastfeaturepoints;
    cv::Mat briefdesc;
    cv::Mat lastbriefdesc;

    _frame->getBriefdesc(briefdesc);
    _lastframe->getBriefdesc(lastbriefdesc);
    _frame->getFeaturepoints(featurepoints);
    _lastframe->getFeaturepoints(lastfeaturepoints);

    std::vector<cv::DMatch> matchepoints;
    if(!briefdesc.empty() && !lastbriefdesc.empty())
    {
        _bfmatcher->match(briefdesc, lastbriefdesc, matchepoints);
    }
    else
    {
        std::cout << "find feature failed" << std::endl;
        return pose;
    }
    if(!matchepoints.empty())
    {
        auto min_max = std::minmax_element(matchepoints.begin(), matchepoints.end(),
                                [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
        float min_dist = min_max.first->distance;
        for(ushort i = 0; i < matchepoints.size(); i++)
        {
            if(matchepoints[i].distance <= std::max(_featurepoint_coe * min_dist, _featurepoint_max))
            {
                _goodmatchepoints.push_back(matchepoints[i]);
            }
        }
        goodmatchpoint_size = _goodmatchepoints.size();
        std::cout << "totally found " << goodmatchpoint_size << " good match point" << std::endl;
    }
    else
    {
        std::cout << "match failed" << std::endl;
        return pose;
    }

    if(goodmatchpoint_size > _goodmatch_thresh)
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setEstimate(pose_estimate);
        vertex_pose->setId(0);
        optimizer.addVertex(vertex_pose);

        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<bool> outliers;
        for(ushort i = 0; i < _goodmatchepoints.size(); i++)
        {
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(
                                _lastframe->get3DPoint(lastfeaturepoints[_goodmatchepoints[i].trainIdx].pt).cast<double>());
            edge->setId(i);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(_frame->get3DPoint(featurepoints[_goodmatchepoints[i].queryIdx].pt).cast<double>());
            edge->setInformation(Eigen::Matrix3d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            outliers.push_back(false);
            optimizer.addEdge(edge);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(_optim_round);
        pose = vertex_pose->estimate().matrix().cast<float>();
        _frame->setPose(pose);
    }
    else
    {
        std::cout << "not enough good match, optimize failed" << std::endl;
        return pose;
    }

    return pose;
}

Eigen::Matrix4f VO::DenseICP_Optimize(Eigen::Matrix4f pose)
{
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_t (new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud_source = _lastframe->getRGBDCloud();
    cloud_target = _frame->getRGBDCloud();
    
    Eigen::Matrix3f rotation = pose.block<3,3>(0,0);
    Eigen::Vector3f translation = pose.block<3,1>(0,3);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(rotation);
    transform.translate(translation);
    
    pcl::transformPointCloud(*cloud_target, *cloud_target_t, transform);

    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target_t);
    icp.setMaximumIterations(5);

    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.align(Final);
    pose = icp.getFinalTransformation();

    return pose;
}


void VO::VOLoop()
{
    std::vector<cv::DMatch> goodmatch;
    while(_vorunning.load())
    {
        if(_vocam->isGrabRdy())
        {
            auto t1 = std::chrono::steady_clock::now();
            std::cout << "<=============================================================>" << std::endl;
            _vocam->setGrabRdyfalse();
            Eigen::Matrix4f pose;
            
            _frame = std::make_shared<Frame>(_vocam);
            _frame->UpdateFrame();
            
            if(!_InitRdy)
            {
                std::cout << "Initializing......" << std::endl;
                _InitRdy = true;
                
                std::cout << "Pose in the world: " << std::endl;
                std::cout << _poses.back() << std::endl; 
            }
            else
            {
                pose = Feature_Optimize();
                // pose = DenseICP_Optimize(pose);
                _poses.push_back(_poses.back()*pose);
                // Point_World = _pose * Point_Cam
                // Cam_pose(in world coordinate) = _pose
                _map->setPose(_poses.back());

                std::cout << "Pose between frame: " << std::endl;
                std::cout << pose << std::endl;
                std::cout << "Pose in the world: " << std::endl;
                std::cout << _poses.back() << std::endl; 
            }
            _lastframe = _frame;
            auto t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout << time_used.count()*1000 << " ms per frame " << std::endl;
        }
    }
}

void VO::VOStop()
{
    _map->MapStop();
    _vorunning.store(false);
    _vothread.join();
}

