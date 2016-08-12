#pragma once

#include <opencv2/opencv.hpp>

class IMainApplication
{
public:
    virtual void initialize() = 0;
    virtual void processFrame(uint32_t frameNumber, cv::Mat newFrame) = 0;
};

