#include "normal_include.h"
#include "g2o_optim.h"
#include <pcl/common/transforms.h>

float inner_cx = 160.5912;
float inner_cy = 120.4792;
float inner_fx = 253.0589;
float inner_fy = 254.1649;

Eigen::Vector3f get3DPoint(cv::Point2f imgpos,cv::Mat& dimage);
void PointcloudTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr before,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr after, Eigen::Matrix4f trans);

int main(int argc,char** argv)
{
    cv::Mat Image1,Image2,dImage1,dImage2;
    std::string reading_data_num1,reading_data_num2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  	pcl::PCDWriter Pclwriter;
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    
    if(argc < 4)
    {
        std::cout << "Need three argument for data to be matched" << std::endl;
        return -1;
    }

    reading_data_num1 = argv[1];
    reading_data_num2 = argv[2];

    Image1 = cv::imread("../savings/rgb/rgb" + reading_data_num1 + ".jpg");
    dImage1 = cv::imread("../savings/depth/depth" + reading_data_num1 + ".jpg");
    Image2 = cv::imread("../savings/rgb/rgb" + reading_data_num2 + ".jpg");
    dImage2 = cv::imread("../savings/depth/depth" + reading_data_num2 + ".jpg");

    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../savings/pointcloud/pointcloud" + reading_data_num1 + ".pcd", *cloud1);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../savings/pointcloud/pointcloud" + reading_data_num2 + ".pcd", *cloud2);
    Eigen::Matrix4f pose;

    if(argv[3][0] == 'f')
    {
        //feature fingding and matching
        std::cout << "feature fingding and matching method" << std::endl;
        cv::Ptr<cv::FeatureDetector> fastdetect;
        cv::Ptr<cv::DescriptorExtractor> briefext;
        cv::Ptr<cv::DescriptorMatcher> bfmatcher;
        std::vector<cv::DMatch> matchepoints;
        std::vector<cv::DMatch> goodmatchepoints;
        fastdetect = cv::FastFeatureDetector::create(50);
        briefext = cv::ORB::create();
        bfmatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        cv::Mat grayImage1;
        std::vector<cv::KeyPoint> featurepoints1;
        cv::Mat briefdesc1;
        std::vector<cv::DMatch> goodmatchepoints1;
        cv::cvtColor(Image1, grayImage1, cv::COLOR_BGR2GRAY);
        fastdetect->detect(grayImage1, featurepoints1);
        briefext->compute(grayImage1, featurepoints1, briefdesc1);
        
        cv::Mat grayImage2;
        std::vector<cv::KeyPoint> featurepoints2;
        cv::Mat briefdesc2;
        std::vector<cv::DMatch> goodmatchepoints2;
        cv::cvtColor(Image2, grayImage2, cv::COLOR_BGR2GRAY);
        fastdetect->detect(grayImage2, featurepoints2);
        briefext->compute(grayImage2, featurepoints2, briefdesc2);


        if(!briefdesc1.empty() && !briefdesc2.empty())
        {
            bfmatcher->match(briefdesc1, briefdesc2, matchepoints);
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
            for(ushort i = 0; i < matchepoints.size(); i++)
            {
                // if(matchepoints[i].distance <= std::max(2.0 * min_dist, 50.0))
                // {
                //     goodmatchepoints.push_back(matchepoints[i]);
                // }
                goodmatchepoints.push_back(matchepoints[i]);
            }
        }
        else
        {
            std::cout << "match failed" << std::endl;
            return -2;
        }

        std::cout << "Found " << goodmatchepoints.size() << " matched points" << std::endl;
        cv::Mat img_match;
        cv::namedWindow("img_match", cv::WINDOW_NORMAL);
        cv::drawMatches(Image1, featurepoints1, Image2, featurepoints2, goodmatchepoints, img_match);
        cv::imshow("img_match", img_match);
        
        // g2o optimization
        SE3 pose_estimate;

        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);

        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setEstimate(pose_estimate);
        vertex_pose->setId(0);
        optimizer.addVertex(vertex_pose);

        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<bool> outliers;
        for(ushort i = 0; i < goodmatchepoints.size(); i++)
        {
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(get3DPoint(featurepoints2[goodmatchepoints[i].trainIdx].pt,dImage2).cast<double>());
            edge->setId(i);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(get3DPoint(featurepoints1[goodmatchepoints[i].queryIdx].pt,dImage1).cast<double>());
            edge->setInformation(Eigen::Matrix3d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            outliers.push_back(false);
            optimizer.addEdge(edge);
        }

        for (int iteration = 0; iteration < 4; ++iteration)
        {
            std::cout << pose_estimate.matrix() << std::endl;
            vertex_pose->setEstimate(pose_estimate);
            optimizer.initializeOptimization();
            optimizer.optimize(10);

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];
                if(outliers[i])
                {
                    e->computeError();
                }
                if(e->chi2() > 5000000000.0)
                {
                    outliers[i] = true;
                    e->setLevel(1);
                }
                else
                {
                    outliers[i] = false;
                    e->setLevel(0);
                }
                if(iteration == 2)
                {
                    //use no kernel after the first iter
                    e->setRobustKernel(nullptr);
                }
            }
            pose_estimate = vertex_pose->estimate();
        }
        
        pose = vertex_pose->estimate().matrix().cast<float>();
        PointcloudTransform(cloud1, new_cloud1, pose);
        *cloud2 += *new_cloud1;
    }


    else if(argv[3][0] == 'i')
    {
        // direct icp from pcl
        std::cout << "direct icp from pcl" << std::endl;
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_t (new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud_source = cloud1;
        cloud_target = cloud2;
        
        icp.setInputSource(cloud_source);
        icp.setInputTarget(cloud_target);
        icp.setMaximumIterations(50);

        pcl::PointCloud<pcl::PointXYZRGB> Final;
        icp.align(Final);
        pose = icp.getFinalTransformation();
    }
    
    std::cout << pose << std::endl;
    
    PointcloudTransform(cloud1, new_cloud1, pose);
    *cloud2 += *new_cloud1;
    Pclwriter.write("../savings/pointcloud/frame_joint.pcd",*cloud2);
    viewer.showCloud(cloud2);

    while (!viewer.wasStopped ())
    {

    }

    return 0;
}


Eigen::Vector3f get3DPoint(cv::Point2f imgpos,cv::Mat& dimage)
{
    Eigen::Matrix<float, 3, 1> temppoint;
    float depth;

    depth = dimage.at<ushort>(imgpos.x,imgpos.y);
    temppoint << (imgpos.x - inner_cx)*depth/inner_fx, (imgpos.y - inner_cy)*depth/inner_fy, depth;

    return temppoint;
}

void PointcloudTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr before,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr after, Eigen::Matrix4f trans)
{
    Eigen::Matrix3f rotation = trans.block<3,3>(0,0);
    Eigen::Vector3f translation = trans.block<3,1>(0,3);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(rotation);
    transform.translate(translation);

    pcl::transformPointCloud(*before, *after, transform);
}