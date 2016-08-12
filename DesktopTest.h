#pragma once

#include "IMainApplication.h"
#include "ColorBasedTargetDetector.h"
#include <opencv2/opencv.hpp>

class DesktopTest : public IMainApplication
{
public:
    virtual void initialize() override;
    virtual void processFrame(uint32_t frameNumber, cv::Mat newFrame) override;

private:
    int hMin = 0; int hMax = 180;
    int sMin = 0; int sMax = 255;
    int vMin = 0; int vMax = 255;

    // These must start as false, otherwise the trackbar won't match the value.
    bool excludeSaturationInHist = false;
    bool enableThreshold = false;
    bool disableImshow = false;

    cv::Rect* selectedTarget = nullptr;
    
    cv::Mat hsvFrame;
    cv::Mat threshFrame;
    cv::Mat histogramRender;
    cv::Mat dbgBackprojFrame;

    // 1 second prune time threshold
    const int64 targetPruneTime = (int64)(cv::getTickFrequency() * 1);

    ColorBasedTargetDetector targetDetector = ColorBasedTargetDetector(this->targetPruneTime, 5);
    ColorBasedTargetDetector::Params detectorParams;

    static void setBoolCallback(int pos, void* userData);
    static void onMouse(int event, int x, int y, int flags, void* userData);
    static void renderHueHistogram(cv::Mat hueHistogram, cv::Mat& histogramRender, int numBins);
};