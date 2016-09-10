#pragma once

#include "IMainApplication.h"
#include "DesktopTest.h"
#include "TimeUtils.h"
#include "FpsCounter.h"
#include "ColorBasedTargetDetector.h"
#include "HeadlessApp.h"

#include <opencv2/opencv.hpp>

int main(int argc, const char* argv[])
{
    IMainApplication* app = (argc > 1 && std::string(argv[1]) == std::string("--with-gui")) ? (IMainApplication*)new DesktopTest() : (IMainApplication*)new HeadlessApp();

    app->initialize();

    FpsCounter fpsCounter = FpsCounter(10);
    cv::VideoCapture capture(0);

    if (!capture.isOpened())
        throw std::exception("Capture not successfully opened!");

    cv::Mat sourceFrame;
    for (uint32_t frameNumber = 0; capture.isOpened(); frameNumber++)
    {
        fpsCounter.addFrameTimestamp();

        capture >> sourceFrame;
        app->processFrame(frameNumber, sourceFrame);

        printf("\r%.1f FPS", fpsCounter.getFps());
    }
}