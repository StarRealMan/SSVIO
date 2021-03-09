#include "FMatch.h"

FMatch::FMatch(const char* voc_dataset, Config::Ptr config)
{
    _voc->load(voc_dataset);
    _Threshold = config->GetParam<int>("Threshold");;
    _NNRatio = config->GetParam<int>("NNRatio");;
}

FMatch::~FMatch()
{

}

void FMatch::MatchByBOW(cv::Mat descriptor_query, cv::Mat descriptor_train, std::vector<cv::DMatch> bow_match)
{
    if(!(descriptor_query.rows > 0 && descriptor_train.rows > 0)) {
        return;
    }

    bow_match.clear();

    DBoW3::BowVector bow_vec_query, bow_vec_train;
    DBoW3::FeatureVector feature_vec_query, feature_vec_train;

    CalBoWVector(descriptor_query, bow_vec_query, feature_vec_query);
    CalBoWVector(descriptor_train, bow_vec_train, feature_vec_train);

    DBoW3::FeatureVector::iterator f_query_begin = feature_vec_query.begin();
    DBoW3::FeatureVector::iterator f_train_begin = feature_vec_train.begin();
    DBoW3::FeatureVector::iterator f_query_end = feature_vec_query.end();
    DBoW3::FeatureVector::iterator f_train_end = feature_vec_train.end();

    while(f_query_begin != f_query_end && f_train_begin != f_train_end)
    {
        if(f_query_begin->first == f_train_begin->first)
        {
            const std::vector<unsigned int> index_query = f_query_begin->second;
            const std::vector<unsigned int> index_train = f_train_begin->second;

            for(int i = 0; i < index_query.size(); ++i)
            {
                const size_t index_1 = index_query[i];
                cv::Mat feat_query = descriptor_query.row(index_1);
                int best_dist_1 = 256;
                int best_dist_2 = 256;
                int best_index = -1;
                for(int j = 0; j < index_train.size(); ++j)
                {
                    const size_t index_2 = index_train[j];
                    cv::Mat feat_train = descriptor_train.row(index_2);
                    // 计算描述子间的距离
                    int dist = CalMatchingScore(feat_query, feat_train);

                    if (dist < best_dist_1) {
                        best_dist_2 = best_dist_1;
                        best_dist_1 = dist;
                        best_index = index_2;
                    }
                    else if (dist < best_dist_2) {
                        best_dist_2 = dist;
                    }
                }

                if(best_dist_1 <= _Threshold) {
                    if (static_cast<float>(best_dist_1) < _NNRatio * static_cast<float>(best_dist_2)) {
                        cv::DMatch m;
                        m.queryIdx = index_1;
                        m.trainIdx = best_index;
                        m.distance = best_dist_1;
                        bow_match.push_back(m);
                    }
                }
            }

            f_query_begin++;
            f_train_begin++;
        }
        else if(f_query_begin->first < f_train_begin->first) {
            f_query_begin = feature_vec_query.lower_bound(f_train_begin->first);
        }
        else {
            f_train_begin = feature_vec_train.lower_bound(f_query_begin->first);
        }
    }
}


void FMatch::CalBoWVector(cv::Mat& descriptor, DBoW3::BowVector& bow_vec, DBoW3::FeatureVector& feat_vec)
{
    bow_vec.clear();
    feat_vec.clear();

    std::vector<cv::Mat> all_desp;
    all_desp.reserve(descriptor.rows);
    for(int i = 0; i < descriptor.rows; i++)
    {
        all_desp.push_back(descriptor.row(i));
    }

    _voc->transform(all_desp, bow_vec, feat_vec, 4);
}

int FMatch::CalMatchingScore(cv::Mat& descriptor_1, cv::Mat& descriptor_2)
{    
    const int *p1 = descriptor_1.ptr<int32_t>();
    const int *p2 = descriptor_2.ptr<int32_t>();

    int dist = 0;

    for(int i = 0; i < 8; ++i, ++p1, ++p2)
    {
        unsigned int v = (*p1) ^ (*p2);
        v = v - ( (v >> 1) & 0x55555555 );
        v = ( v & 0x33333333 ) + ( (v >> 2) & 0x33333333 );
        dist += ( ( (v + (v >> 4)) & 0xF0F0F0F ) * 0x1010101 ) >> 24;
    }

    return dist;
}