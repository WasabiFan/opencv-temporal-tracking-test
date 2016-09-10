#pragma once

#include "IMainApplication.h"
#include "ColorBasedTargetDetector.h"
#include "AppParamsManager.h"
#include "RobotComms.h"
#include <opencv2/opencv.hpp>

class HeadlessApp : public IMainApplication
{
public:
    HeadlessApp();
    ~HeadlessApp();

    virtual void initialize() override;
    virtual void processFrame(uint32_t frameNumber, cv::Mat newFrame) override;

private:

    AppParams appParams;
    
	bool frameSizeSet = false;

    cv::Mat hsvFrame;
    cv::Mat threshFrame;

    // 1 second prune time threshold
    const int64 targetPruneTime = (int64)(cv::getTickFrequency() * 1);

    const string commServerAddress = "roborio-488-frc.local";
    const unsigned short commServerPort = 3000;

    RobotComms* robotComms;

    ColorBasedTargetDetector targetDetector = ColorBasedTargetDetector(this->targetPruneTime, 5);
};
