#include "Odometry.h"
#include "Local.h"
#include "Loop.h"
#include "Viewer.h"
#include "Config.h"
#include "Frame.h"
#include "MapPoint.h"
#include "Local.h"
#include "IMU.h"
#include "Optimizer.h"

Config::Ptr config(new Config("../config/default_conf.yaml"));

float Frame::_InnerCx = config->GetParam<float>("InnerCx");
float Frame::_InnerCy = config->GetParam<float>("InnerCy");
float Frame::_InnerFx = config->GetParam<float>("InnerFx");
float Frame::_InnerFy = config->GetParam<float>("InnerFy");
float Frame::_DepthScale = config->GetParam<float>("DepthScale")/65535.0;
float Frame::_InvInnerFx = 1/Frame::_InnerFx;
float Frame::_InvInnerFy = 1/Frame::_InnerFy;
int Frame::_MaxMatchPointThres = config->GetParam<int>("MaxMatchPointThres");
int Frame::_MinMatchPointThres = config->GetParam<int>("MinMatchPointThres");
int Frame::_MaxFramesBetween = config->GetParam<int>("MaxFramesBetween");
int Frame::_MinFramesBetween = config->GetParam<int>("MinFramesBetween");

float OdomOptimizer::_IMUGain = config->GetParam<float>("IMUGain");
float OdomOptimizer::_Chi2Thresh = config->GetParam<float>("Chi2Thresh");
float OdomOptimizer::_ZAxisInfo = config->GetParam<float>("ZAxisInfo");

float LocalOptimizer::_Chi2Thresh = config->GetParam<float>("LocalChi2Thresh");
float LocalOptimizer::_ZAxisInfo = config->GetParam<float>("ZAxisInfo");

int Frame::_key_frame_num = 0;
int MapPoint::_map_point_num = 0;
