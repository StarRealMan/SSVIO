#ifndef __FMATCH_H__
#define __FMATCH_H__

#include "DBoW3/DBoW3.h"

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Config.h"

class FMatch
{
public:
    typedef std::shared_ptr<FMatch> Ptr;

    FMatch(const char* voc_dataset, Config::Ptr config);
    ~FMatch();

    void MatchByBOW(cv::Mat desscriptor_query, cv::Mat desscriptor_train, std::vector<cv::DMatch> bow_match);
    void CalBoWVector(cv::Mat& descriptor, DBoW3::BowVector& bowVec, DBoW3::FeatureVector& featVec);
    int CalMatchingScore(cv::Mat& desp1, cv::Mat& desp2);

private:
    std::shared_ptr<DBoW3::Vocabulary> _voc;
    int _Threshold;
    float _NNRatio;

};


#endif