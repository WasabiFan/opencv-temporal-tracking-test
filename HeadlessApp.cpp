#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>

#include "TimeUtils.h"
#include "ColorBasedTargetDetector.h"
#include "HeadlessApp.h"

using namespace cv;

void HeadlessApp::initialize()
{
    // TODO: Load settings
}

void HeadlessApp::processFrame(uint32_t frameNumber, cv::Mat newFrame)
{
    cvtColor(newFrame, hsvFrame, CV_BGR2HSV);

    if (enableThreshold)
        inRange(hsvFrame, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), threshFrame);

    if (targetDetector.hasTargetTraining())
    {
        targetDetector.updateTracking(hsvFrame, cv::getTickCount(), detectorParams, threshFrame);
    }
    else
    {
        // TODO: Log error
    }
}
