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
int Frame::_MaxGoodPointThres = config->GetParam<int>("MaxGoodPointThres");
int Frame::_MaxFramesBetween = config->GetParam<int>("MaxFramesBetween");
int Frame::_MinFramesBetween = config->GetParam<int>("MinFramesBetween");
double Frame::_DepthScale = config->GetParam<float>("DepthScale")/65535.0;
float OdomOptimizer::_IMUGain = config->GetParam<float>("IMUGain");
float Frame::_InvInnerFx = 1/Frame::_InnerFx;
float Frame::_InvInnerFy = 1/Frame::_InnerFy;
int Frame::_key_frame_point_num = 0;
int MapPoint::_map_point_num = 0;