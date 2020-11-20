#include "Frame.h"

Frame::Frame(Xtion_Camera::Ptr camera)
{
    _framecam = camera;
    _fastdetect = cv::FastFeatureDetector::create(50);
    _briefext = cv::ORB::create();
    _bfmatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

Frame::~Frame()
{

}

void Frame::getGoodMatch(std::vector<cv::DMatch>& goodmatchepoints)
{
    std::lock_guard<std::mutex> lck(_match_mtx);
    goodmatchepoints = _goodmatchepoints;
}

Eigen::Matrix4f Frame::getPose()
{
    std::lock_guard<std::mutex> lck(_pose_mtx);
    return _pose;
}

void Frame::UpdateFrame()
{
    _rgbframe = _framecam->getRGBImage();
    _dframe = _framecam->getDImage();
    _rgbcloud = _framecam->getRGBCloud();
    _goodmatchepoints.clear();
    cv::Mat grayframe;
    cv::cvtColor(_rgbframe, grayframe, cv::COLOR_BGR2GRAY);
    _fastdetect->detect(grayframe, _featurepoints);
    _briefext->compute(grayframe, _featurepoints, _briefdesc);
}

bool Frame::Optimize(Frame::Ptr lastframe)
{
    Eigen::Matrix4f pose_estimate;

    std::vector<cv::DMatch> matchepoints;
    if(!_briefdesc.empty() && !(lastframe->_briefdesc.empty()))
    {
        _bfmatcher->match(_briefdesc, lastframe->_briefdesc, matchepoints);
    }
    else
    {
        std::cout << "find feature failed" << std::endl;
        return -1;
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
    }
    else
    {
        std::cout << "match failed" << std::endl;
        return -2;
    }
    
    // pose_estimate
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
    std::vector<EdgeProjectionPoseOnly *> edges;
    for(int i = 0; i < _goodmatchepoints.size(); i++) {
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(Vec3(_goodmatchepoints(i,lastframe)->get3DPoint()));
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(Vec3(_goodmatchepoints(i,lastframe)->get3DPoint()));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for(int iteration = 0; iteration < 4; ++iteration)
    {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        for(size_t i = 0; i < edges.size(); ++i)
        {
            auto e = edges[i];
            if(features[i]->is_outlier_)
            {
                e->computeError();
            }
            if(e->chi2() > chi2_th)
            {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            }
            else
            {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            }
            if(iteration == 2)
            {
                e->setRobustKernel(nullptr);
            }
        }
    }


    _pose = pose_estimate;
}


