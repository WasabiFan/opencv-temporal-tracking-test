#pragma once

#include "IMainApplication.h"
#include "ColorBasedTargetDetector.h"
#include "AppParamsManager.h"
#include "RobotComms.h"
#include <opencv2/opencv.hpp>

class DesktopTest : public IMainApplication
{
public:
    DesktopTest();
    ~DesktopTest();

    virtual void initialize() override;
    virtual void processFrame(uint32_t frameNumber, cv::Mat newFrame) override;

private:
    AppParams appParams;

    // These must start as false, otherwise the trackbar won't match the value.
    bool excludeSaturationInHist = false;
    bool disableImshow = false;
	bool frameSizeSet = false;

    cv::Rect* selectedTarget = nullptr;
    
    cv::Mat hsvFrame;
    cv::Mat threshFrame;
    cv::Mat histogramRender;
    cv::Mat dbgBackprojFrame;

    const string commServerAddress = "roborio-488-frc.local";
    const unsigned short commServerPort = 3000;

    // 1 second prune time threshold
    const int64 targetPruneTime = (int64)(cv::getTickFrequency() * 1);

    ColorBasedTargetDetector targetDetector = ColorBasedTargetDetector(this->targetPruneTime, 5);
    RobotComms* robotComms = nullptr;

    static void setBoolCallback(int pos, void* userData);
    static void onMouse(int event, int x, int y, int flags, void* userData);
    static void renderHueHistogram(cv::Mat hueHistogram, cv::Mat& histogramRender, int numBins);
};
