#include "Visual_Odometry.h"

VO::VO(Xtion_Camera::Ptr camera)
{
    _vocam = camera;

    _poses.push_back(Eigen::Matrix4f::Identity());
    _bfmatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    _vorunning.store(true);
    _vothread = std::thread(std::bind(&VO::VOLoop,this));
}

VO::~VO()
{

}

Eigen::Matrix4f VO::Optimize()
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
        double min_dist = min_max.first->distance;
        for (int i = 0; i < matchepoints.size(); i++)
        {
            if (matchepoints[i].distance <= std::max(2 * min_dist, 30.0))
            {
                _goodmatchepoints.push_back(matchepoints[i]);
            }
        }
        goodmatchpoint_size = _goodmatchepoints.size();
        std::cout << "Totally found " << goodmatchpoint_size << " good match point" << std::endl;
    }
    else
    {
        std::cout << "match failed" << std::endl;
        return pose;
    }

    if(goodmatchpoint_size > 30)
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(SE3());
        optimizer.addVertex(vertex_pose);

        int index = 1;
        for(int i = 0; i < _goodmatchepoints.size(); i++)
        {
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(
                                _lastframe->get3DPoint(lastfeaturepoints[_goodmatchepoints[i].trainIdx].pt).cast<double>());
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(_frame->get3DPoint(featurepoints[_goodmatchepoints[i].queryIdx].pt).cast<double>());
            edge->setInformation(Eigen::Matrix3d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            optimizer.addEdge(edge);
            index++;
        }

        vertex_pose->setEstimate(pose_estimate);
        optimizer.initializeOptimization();
        optimizer.optimize(30);
        
        pose = pose_estimate.matrix().cast<float>();
        _frame->setPose(pose);
        std::cout << pose << std::endl;
    }
    else
    {
        std::cout << "not enough good match, optimize failed" << std::endl;
        return pose;
    }

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
            // std::cout << "vo start" << std::endl;
            _vocam->setGrabRdyfalse();
            Eigen::Matrix4f pose;
            
            //_map(new Map)
            _frame = std::make_shared<Frame>(_vocam);
            _frame->UpdateFrame();
            std::cout << "<=============================================================>" << std::endl;
            
            if(!_InitRdy)
            {
                std::cout << "Initializing......" << std::endl;
                _InitRdy = true;
            }
            else
            {
                pose = Optimize();
                _poses.push_back(_poses[-1]*pose);
                // Map
            }
            _lastframe = _frame;
            // std::cout << "vo ok" << std::endl;
            auto t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout << time_used.count()*1000 << " ms per frame " << std::endl;
        }
    }
}

void VO::VOStop()
{
    _vorunning.store(false);
    _vothread.join();
}

