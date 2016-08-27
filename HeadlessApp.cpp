#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>

#include "TimeUtils.h"
#include "ColorBasedTargetDetector.h"
#include "HeadlessApp.h"

using namespace cv;

HeadlessApp::HeadlessApp()
{
    robotComms = new RobotComms(this->commServerAddress, this->commServerPort);
}

HeadlessApp::~HeadlessApp()
{
    delete robotComms;
}

void HeadlessApp::initialize()
{
    AppParamsManager::loadParams("config.xml", appParams);
    if (!this->appParams.targetHistogram.empty())
        this->targetDetector.updateTargetHistogram(appParams.targetHistogram);
}

void HeadlessApp::processFrame(uint32_t frameNumber, cv::Mat newFrame)
{
    cvtColor(newFrame, hsvFrame, CV_BGR2HSV);

    if (appParams.enableHsvThreshold)
        inRange(hsvFrame, Scalar(appParams.threshHMin, appParams.threshSMin, appParams.threshVMin), Scalar(appParams.threshHMax, appParams.threshSMax, appParams.threshVMax), threshFrame);

    if (targetDetector.hasTargetTraining())
    {
        targetDetector.updateTracking(hsvFrame, cv::getTickCount(), appParams, threshFrame);
        robotComms->sendTrackedTargets(targetDetector.getTrackedTargets());
	}
    else
    {
        // TODO: Log error
    }
}
