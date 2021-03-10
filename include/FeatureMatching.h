
#ifndef __FEATUREMATCHING_H__
#define __FEATUREMATCHING_H__

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "DBoW3/DBoW3.h"

#include <memory>

#include "Config.h"

class FeatureMatching {

public:
typedef std::shared_ptr<FeatureMatching> Ptr;
    // 默认构造函数,读取bow词典模型并设置阈值
    FeatureMatching( const char* voc_dataset , Config::Ptr config);

    // 默认析构函数
    ~FeatureMatching();

    // 利用暴力匹配法进行特征匹配
    bool MatchByBruteForce( cv::Mat& desp1, cv::Mat& desp2, std::vector<cv::DMatch>& matches );

    // 利用基于词袋模型的方法进行特征匹配
    bool MatchByDBoW( cv::Mat& desp1, cv::Mat& desp2, std::vector<cv::DMatch>& matches );

    // 利用投影法进行特征匹配
    bool MatchByProject( std::vector<cv::Point3d>& vp3d1, cv::Mat& desp1, 
        std::vector<cv::Point2d>& vp2d2, cv::Mat& desp2, double& radian, 
        cv::Mat& K, cv::Mat& R, cv::Mat& t, std::vector<cv::DMatch>& matches );

private:

    // 将描述子转换为bow向量
    void ComputeBoWVector( cv::Mat& desp, DBoW3::BowVector& bowVec, DBoW3::FeatureVector& featVec);

    // 计算两个描述子间的匹配分数
    int ComputeMatchingScore( cv::Mat& desp1, cv::Mat& desp2 ); 

    // 词袋模型
    std::shared_ptr<DBoW3::Vocabulary> mVoc_ = nullptr;

    // bow向量,包含id和权重
    DBoW3::BowVector mBowVec1_, mBowVec2_;

    // feature向量,包含在bow树中的id和特征的id
    DBoW3::FeatureVector mFeatVec1_, mFeatVec2_;

    // 最近邻比例法阈值条件
    int mTh_;
    float mNNRatio_;
};

#endif