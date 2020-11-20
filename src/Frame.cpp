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

void Frame::getFeaturepoints(std::vector<cv::KeyPoint>& featurepoints)
{
    std::lock_guard<std::mutex> lck(_featurepoints_mtx);
    featurepoints = _featurepoints;
}

void Frame::getBriefdesc(cv::Mat& briefdesc)
{
    std::lock_guard<std::mutex> lck(_briefdesc_mtx);
    briefdesc = _briefdesc;
}

Eigen::Vector3f Frame::get3DPoint(cv::Point2f imgpos)
{
    Eigen::Matrix<float, 3, 1> temppoint;
    float depth;

    depth = _dframe.at<ushort>(imgpos.x,imgpos.y);
    temppoint << (imgpos.x - _inner_cx)*depth*_inv_inner_fx, (imgpos.y - _inner_cy)*depth*_inv_inner_fy, depth;

    return temppoint;
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
   
        if(_goodmatchepoints.size() > 30)
    {
        SE3 pose_estimate;
        std::vector<cv::KeyPoint> lastfeaturepoints;
        lastframe->getFeaturepoints(lastfeaturepoints);
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

        std::cout << "last points " << lastfeaturepoints.size() << std::endl;
        std::cout << "this points " << _featurepoints.size() << std::endl;

        for(int i = 0; i < _goodmatchepoints.size(); i++) {
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(get3DPoint(lastfeaturepoints[_goodmatchepoints[i].trainIdx].pt).cast<double>());
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(get3DPoint(_featurepoints[_goodmatchepoints[i].queryIdx].pt).cast<double>());
            edge->setInformation(Eigen::Matrix3d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
            
            std::cout << "last index " << _goodmatchepoints[i].queryIdx << std::endl;
            std::cout << "this index " << _goodmatchepoints[i].queryIdx << std::endl;
        }
        vertex_pose->setEstimate(pose_estimate);
        optimizer.initializeOptimization();
        optimizer.optimize(30);

        _pose  = pose_estimate.matrix().cast<float>();
        std::cout << _pose << std::endl;
    }
    else
    {
        _pose = lastframe->getPose();
        std::cout << "not enough good match, optimize failed" << std::endl;
        return -3;
    }

    return 0;
}


