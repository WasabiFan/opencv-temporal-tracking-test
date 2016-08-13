#pragma once

#include "IMainApplication.h"
#include "ColorBasedTargetDetector.h"
#include "AppParamsManager.h"
#include <opencv2/opencv.hpp>

class HeadlessApp : public IMainApplication
{
public:
    virtual void initialize() override;
    virtual void processFrame(uint32_t frameNumber, cv::Mat newFrame) override;

private:

    AppParams appParams;
    
    cv::Mat hsvFrame;
    cv::Mat threshFrame;

    // 1 second prune time threshold
    const int64 targetPruneTime = (int64)(cv::getTickFrequency() * 1);

    ColorBasedTargetDetector targetDetector = ColorBasedTargetDetector(this->targetPruneTime, 5);
    ColorBasedTargetDetector::Params detectorParams;
};