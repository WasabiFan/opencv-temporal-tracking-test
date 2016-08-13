
#include "AppParamsManager.h"
#include "opencv2/opencv.hpp"

/*
cv::Mat targetHistogram;

bool enableHsvThreshold = false;
int threshHMin = 0; int threshHMax = 180;
int threshSMin = 0; int threshSMax = 255;
int threshVMin = 0; int threshVMax = 255;

int blurSize = 31;
int blurSigma = 15;
int toZeroThresh = 10;

*/

bool AppParamsManager::loadParams(const std::string sourceFile, AppParams& outParams)
{
    cv::FileStorage file = cv::FileStorage(sourceFile, cv::FileStorage::READ);
    if (!file.isOpened())
        return false;

    file["targetHistogram"] >> outParams.targetHistogram;
    file["enableHsvThreshold"] >> outParams.enableHsvThreshold;
    file["threshHMin"] >> outParams.threshHMin;
    file["threshHMax"] >> outParams.threshHMax;
    file["threshSMin"] >> outParams.threshSMin;
    file["threshSMax"] >> outParams.threshSMax;
    file["threshVMin"] >> outParams.threshVMin;
    file["threshVMax"] >> outParams.threshVMax;

    file["blurSize"] >> outParams.blurSize;
    file["blurSigma"] >> outParams.blurSigma;
    file["toZeroThresh"] >> outParams.toZeroThresh;

    return true;
}

bool AppParamsManager::saveParams(const std::string destFile, AppParams params)
{
    cv::FileStorage file = cv::FileStorage(destFile, cv::FileStorage::WRITE);
    if (!file.isOpened())
        return false;

    file << "targetHistogram" << params.targetHistogram;
    file << "enableHsvThreshold" << params.enableHsvThreshold;
    file << "threshHMin" << params.threshHMin;
    file << "threshHMax" << params.threshHMax;
    file << "threshSMin" << params.threshSMin;
    file << "threshSMax" << params.threshSMax;
    file << "threshVMin" << params.threshVMin;
    file << "threshVMax" << params.threshVMax;

    file << "blurSize" << params.blurSize;
    file << "blurSigma" << params.blurSigma;
    file << "toZeroThresh" << params.toZeroThresh;

    return true;
}
