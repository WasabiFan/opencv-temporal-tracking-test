#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <iostream>

#include "TimeUtils.h"
#include "ColorBasedTargetDetector.h"
#include "DesktopTest.h"
#include "AppParamsManager.h"
//#include "RobotComms.h"

using namespace cv;


#define CAPTURE_CHECKPOINT "Capture"
#define THRESH_CHECKPOINT "Threshold"
#define CVT_COLOR_CHECKPOINT "Cvt color"
#define BACK_PROJECT_CHECKPOINT "Back project"
#define CAM_SHIFT_CHECKPOINT "Cam shift"
#define BLOB_DETECT_CHECKPOINT "Blob detect"
#define RENDER_CHECKPOINT "Render GUI"

void DesktopTest::setBoolCallback(int pos, void* userData)
{
    *(bool*)userData = pos != 0;
}

void DesktopTest::onMouse(int event, int x, int y, int flags, void* userData)
{
    Rect** selectedTargetRef = (Rect**)userData;
    if (event == EVENT_LBUTTONDOWN)
    {
        if (*selectedTargetRef != nullptr)
            delete *selectedTargetRef;

        *selectedTargetRef = new Rect(x, y, 0, 0);
    }
    else if (event == EVENT_LBUTTONUP && *selectedTargetRef != nullptr)
    {
        Rect* selectedTarget = *selectedTargetRef;

        if(x > selectedTarget->x)
            selectedTarget->width = x - selectedTarget->x;
        else if (x < selectedTarget->x)
        {
            selectedTarget->width = selectedTarget->x - x;
            selectedTarget->x = x;
        }

        if (y > selectedTarget->y)
            selectedTarget->height = y - selectedTarget->y;
        else if (y < selectedTarget->y)
        {
            selectedTarget->height = selectedTarget->y - y;
            selectedTarget->y = y;
        }
    }
}

void DesktopTest::renderHueHistogram(Mat hueHistogram, Mat& histogramRender, int numBins)
{
    int binWidth = histogramRender.cols / numBins;
    Mat buf(1, numBins, CV_8UC3);

    for (int i = 0; i < numBins; i++)
        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / numBins), 255, 255);

    cvtColor(buf, buf, COLOR_HSV2BGR);

    for (int i = 0; i < numBins; i++)
    {
        int val = saturate_cast<int>(hueHistogram.at<float>(i)*histogramRender.rows / 255);
        rectangle(histogramRender, Point(i * binWidth, histogramRender.rows),
            Point((i + 1) * binWidth, histogramRender.rows - val),
            Scalar(buf.at<Vec3b>(i)), -1, 8);
    }
}

DesktopTest::DesktopTest()
{
    robotComms = new RobotComms(this->commServerAddress, this->commServerPort);
}

DesktopTest::~DesktopTest()
{
    delete robotComms;
}

void DesktopTest::initialize()
{
    // TODO: Do something if this fails
    AppParamsManager::loadParams("config.xml", this->appParams);
    if (!this->appParams.targetHistogram.empty())
        this->targetDetector.updateTargetHistogram(appParams.targetHistogram);

    /*TimerManager timers;
    timers.addCheckpoint(CAPTURE_CHECKPOINT);
    timers.addCheckpoint(CVT_COLOR_CHECKPOINT, CAPTURE_CHECKPOINT);
    timers.addCheckpoint(THRESH_CHECKPOINT, CVT_COLOR_CHECKPOINT);
    timers.addCheckpoint(BACK_PROJECT_CHECKPOINT, THRESH_CHECKPOINT);
    timers.addCheckpoint(BLOB_DETECT_CHECKPOINT, BACK_PROJECT_CHECKPOINT);
    timers.addCheckpoint(CAM_SHIFT_CHECKPOINT, BLOB_DETECT_CHECKPOINT);
    timers.addCheckpoint(RENDER_CHECKPOINT, CAM_SHIFT_CHECKPOINT);*/

    namedWindow("Source", CV_WINDOW_NORMAL);
    namedWindow("Thresh", CV_WINDOW_NORMAL);
    namedWindow("Backprojected frame", CV_WINDOW_NORMAL);
    namedWindow("Histogram render", CV_WINDOW_NORMAL);

    namedWindow("Config", CV_WINDOW_NORMAL);
    createTrackbar("H min", "Config", &appParams.threshHMin, 180);
    createTrackbar("H max", "Config", &appParams.threshHMax, 180);
    createTrackbar("S min", "Config", &appParams.threshSMin, 255);
    createTrackbar("S max", "Config", &appParams.threshSMax, 255);
    createTrackbar("V min", "Config", &appParams.threshVMin, 255);
    createTrackbar("V max", "Config", &appParams.threshVMax, 255);
    createTrackbar("Disable sat", "Config", nullptr, 1, &(DesktopTest::setBoolCallback), &excludeSaturationInHist);
    createTrackbar("Enable thresh", "Config", nullptr, 1, &(DesktopTest::setBoolCallback), &appParams.enableHsvThreshold);
    createTrackbar("Disable imshow", "Config", nullptr, 1, &(DesktopTest::setBoolCallback), &disableImshow);

    
    setMouseCallback("Source", this->onMouse, &selectedTarget);

    createTrackbar("Blur size", "Config", &appParams.blurSize, 60);
    createTrackbar("Blur s", "Config", &appParams.blurSigma, 60);
    createTrackbar("Thresh", "Config", &appParams.toZeroThresh, 100);
}

void DesktopTest::processFrame(uint32_t frameNumber, cv::Mat newFrame)
{
    //timers.markCheckpoint(CAPTURE_CHECKPOINT);

    //medianBlur(sourceFrame, sourceFrame, 5);
    cvtColor(newFrame, hsvFrame, CV_BGR2HSV);

    //timers.markCheckpoint(CVT_COLOR_CHECKPOINT);

    if (appParams.enableHsvThreshold)
        inRange(hsvFrame, Scalar(appParams.threshHMin, appParams.threshSMin, appParams.threshVMin), Scalar(appParams.threshHMax, appParams.threshSMax, appParams.threshVMax), threshFrame);

    // timers.markCheckpoint(THRESH_CHECKPOINT);

    if (selectedTarget != nullptr && selectedTarget->area() > 0)
    {
        // TODO: Transition selection code to smart_ptr
        //printf("Updating target histogram with rect (x: %d, y: %d) (w: %d, h: %d)\r\n", selectedTarget->x, selectedTarget->y, selectedTarget->width, selectedTarget->height);
        Mat sourceRoi = Mat(hsvFrame, *selectedTarget);
        Mat maskRoi = threshFrame.empty() ? Mat() : Mat(threshFrame, *selectedTarget);

        Mat calculatedHistogram;
        ColorBasedTargetDetector::calculateHistFromTarget(calculatedHistogram, excludeSaturationInHist ? TRACKER_HUE : TRACKER_HUE_SAT, sourceRoi, maskRoi);
        
        appParams.targetHistogram = calculatedHistogram;
        this->targetDetector.updateTargetHistogram(appParams.targetHistogram);

        // TODO: Re-write histogram render for 2d histogram with sat
        //std::cout << targetColorHistogram << std::endl;

        if (excludeSaturationInHist)
        {
            histogramRender = Mat::zeros(200, calculatedHistogram.rows * 10, CV_8UC3);
            this->renderHueHistogram(calculatedHistogram, histogramRender, calculatedHistogram.rows);
            imshow("Histogram render", histogramRender);
        }

        delete selectedTarget;
        selectedTarget = nullptr;
    }


    if (targetDetector.hasTargetTraining())
    {
        targetDetector.updateTracking(hsvFrame, cv::getTickCount(), appParams, threshFrame);

        robotComms->sendTrackedTargets(targetDetector.getTrackedTargets());

        if (!disableImshow)
        {
            for (auto trackedTarget : targetDetector.getTrackedTargets())
            {
                if (trackedTarget->lastTrackedPose.size.area() > 0)
                    ellipse(newFrame, trackedTarget->lastTrackedPose, Scalar(255, 0, 255), 1);
            }
        }

        //timers.markCheckpoint(RENDER_CHECKPOINT);

        if (!disableImshow)
        {
            targetDetector.getLastBackprojFrame(dbgBackprojFrame);
            imshow("Backprojected frame", dbgBackprojFrame);
        }
    }
    else
    {
        /*timers.markCheckpoint(BACK_PROJECT_CHECKPOINT);
        timers.markCheckpoint(BLOB_DETECT_CHECKPOINT);
        timers.markCheckpoint(CAM_SHIFT_CHECKPOINT);
        timers.markCheckpoint(RENDER_CHECKPOINT);*/
    }

    /* timers.stop();
    timers.printTableRow();*/

    if (!disableImshow || !targetDetector.hasTargetTraining())
        imshow("Source", newFrame);

    if (!disableImshow && appParams.enableHsvThreshold)
        imshow("Thresh", threshFrame);

    if (waitKey(1) == 's')
    {
        // TODO: Use global struct with environment paths
        if (AppParamsManager::saveParams("config.xml", this->appParams))
            printf("\r\nSuccessfully saved params\r\n");
        else
            printf("\r\nError while saving params!\n");
    }
}
