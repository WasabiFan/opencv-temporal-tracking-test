#pragma once

#include "ColorBasedTargetDetector.h"
#include "opencv2/opencv.hpp"

struct AppParams : public ColorBasedTargetDetector::Params
{
    cv::Mat targetHistogram;

    bool enableHsvThreshold = false;
    int threshHMin = 0; int threshHMax = 180;
    int threshSMin = 0; int threshSMax = 255;
    int threshVMin = 0; int threshVMax = 255;
};

class AppParamsManager
{
public:
    static bool loadParams(const std::string sourceFile, AppParams& outParams);
    static bool saveParams(const std::string destFile, AppParams params);
};

