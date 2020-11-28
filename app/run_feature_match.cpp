#include "normal_include.h"

int main(int argc,char** argv)
{
    cv::Mat Image1,Image2;
    std::string reading_data_num1,reading_data_num2;
    
    if(argc < 2)
    {
        std::cout << "Need two argument for data to be matched" << std::endl;
        return -1;
    }

    reading_data_num1 = argv[1];
    reading_data_num2 = argv[2];

    Image1 = cv::imread("../savings/rgb/rgb"+ reading_data_num1 + ".jpg");
    Image2 = cv::imread("../savings/rgb/rgb"+ reading_data_num2 + ".jpg");


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
        for (int i = 0; i < matchepoints.size(); i++)
        {
            if (matchepoints[i].distance <= std::max(2 * min_dist, 30.0))
            {
                goodmatchepoints.push_back(matchepoints[i]);
            }
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
    cv::drawMatches(grayImage1, featurepoints1, grayImage2, featurepoints2, goodmatchepoints, img_match);
    cv::imshow("img_match", img_match);
    cv::imwrite("match.jpg", img_match);
    cv::waitKey(0);
    return 0;
}