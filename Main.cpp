#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <iostream>

#include "TimeUtils.h"
#include "FpsCounter.h"

using namespace cv;

/*
    NOTES:

    - Dilate and erode dramatically decrease blob time
    - Can do blobs every few frames to minimize slowdown
    - Blur removed to speed up frame time. Can be added back if desired.
*/

#define CAPTURE_CHECKPOINT "Capture"
#define THRESH_CHECKPOINT "Threshold"
#define CVT_COLOR_CHECKPOINT "Cvt color"
#define BACK_PROJECT_CHECKPOINT "Back project"
#define CAM_SHIFT_CHECKPOINT "Cam shift"
#define BLOB_DETECT_CHECKPOINT "Blob detect"
#define RENDER_CHECKPOINT "Render GUI"

struct TargetBoundaryInfo
{
    std::shared_ptr<Rect> targetBounds;
    RotatedRect lastTrackedPose;
    int64 lastDetectedTime = -1;
};

bool operator==(KeyPoint const& lhs, KeyPoint const& rhs)
{
    return lhs.pt == rhs.pt
        && lhs.angle == rhs.angle
        && lhs.size == rhs.size
        && lhs.response == rhs.response
        && lhs.octave == rhs.octave
        && lhs.class_id == rhs.class_id;
}

static void setBoolCallback(int pos, void* userData)
{
    *(bool*)userData = pos != 0;
}

static void onMouse(int event, int x, int y, int flags, void* userData)
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

void renderHueHistogram(Mat hueHistogram, Mat& histogramRender, int numBins)
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

void updateTargetCorrelation(std::vector<std::shared_ptr<TargetBoundaryInfo>>& targets, std::vector<KeyPoint> detectedBlobs, int64 currentTime, int64 targetPruneTimeThresh)
{
    //TODO: Add logging
    std::vector<KeyPoint> unpairedBlobs = detectedBlobs;
    std::vector<std::shared_ptr<TargetBoundaryInfo>> unpairedTargets = targets;
    for (KeyPoint blob : detectedBlobs)
    {
        for (std::shared_ptr<TargetBoundaryInfo> target : unpairedTargets)
        {
            // TODO: Check for overlap with keypoint diameter as well
            if (target->targetBounds->contains(blob.pt))
            {
                target->lastDetectedTime = currentTime;
                unpairedTargets.erase(std::remove(unpairedTargets.begin(), unpairedTargets.end(), target), unpairedTargets.end());

                // KeyPoint doesn't define a == operator
                for (int i = 0; i < unpairedBlobs.size(); i++)
                    if (unpairedBlobs[i] == blob)
                        unpairedBlobs.erase(std::next(unpairedBlobs.begin(), i));

                break;
            }
        }
    }

    for (std::shared_ptr<TargetBoundaryInfo> unpairedTarget : unpairedTargets)
    {
        if (currentTime - unpairedTarget->lastDetectedTime >= targetPruneTimeThresh)
            targets.erase(std::remove(targets.begin(), targets.end(), unpairedTarget), targets.end());
    }

    for (KeyPoint unpairedBlob : unpairedBlobs)
    {
        auto newTarget = std::make_shared<TargetBoundaryInfo>();
        newTarget->lastDetectedTime = currentTime;
        newTarget->targetBounds = std::make_shared<Rect>();
        newTarget->targetBounds->x = (int)unpairedBlob.pt.x;
        newTarget->targetBounds->y = (int)unpairedBlob.pt.y;
        newTarget->targetBounds->width = (int)unpairedBlob.size;
        newTarget->targetBounds->height = (int)unpairedBlob.size;

        targets.push_back(newTarget);
    }
}

int main()
{
    int hMin = 0, hMax = 180,
        sMin = 0, sMax = 255,
        vMin = 0, vMax = 255;

    // These must start as false, otherwise the trackbar won't match the value.
    bool includeSatInHist = false;
    bool enableThreshold = false;
    bool disableImshow = false;

    TimerManager timers;
    timers.addCheckpoint(CAPTURE_CHECKPOINT);
    timers.addCheckpoint(CVT_COLOR_CHECKPOINT, CAPTURE_CHECKPOINT);
    timers.addCheckpoint(THRESH_CHECKPOINT, CVT_COLOR_CHECKPOINT);
    timers.addCheckpoint(BACK_PROJECT_CHECKPOINT, THRESH_CHECKPOINT);
    timers.addCheckpoint(BLOB_DETECT_CHECKPOINT, BACK_PROJECT_CHECKPOINT);
    timers.addCheckpoint(CAM_SHIFT_CHECKPOINT, BLOB_DETECT_CHECKPOINT);
    timers.addCheckpoint(RENDER_CHECKPOINT, CAM_SHIFT_CHECKPOINT);

    FpsCounter fpsCounter = FpsCounter(10);

    VideoCapture capture(0);
    namedWindow("Source", CV_WINDOW_NORMAL);
    namedWindow("Thresh", CV_WINDOW_NORMAL);
    namedWindow("Backprojected frame", CV_WINDOW_NORMAL);
    namedWindow("Histogram render", CV_WINDOW_NORMAL);

    namedWindow("Config", CV_WINDOW_NORMAL);
    createTrackbar("H min", "Config", &hMin, 180);
    createTrackbar("H max", "Config", &hMax, 180);
    createTrackbar("S min", "Config", &sMin, 255);
    createTrackbar("S max", "Config", &sMax, 255);
    createTrackbar("V min", "Config", &vMin, 255);
    createTrackbar("V max", "Config", &vMax, 255);
    createTrackbar("Enable sat", "Config", nullptr, 1, setBoolCallback, &includeSatInHist);
    createTrackbar("Enable thresh", "Config", nullptr, 1, setBoolCallback, &enableThreshold);
    createTrackbar("Disable imshow", "Config", nullptr, 1, setBoolCallback, &disableImshow);

    Rect* selectedTarget = nullptr;
    setMouseCallback("Source", onMouse, &selectedTarget);

    // TODO: Tune blob params
    cv::SimpleBlobDetector::Params blobParams;
    blobParams.minDistBetweenBlobs = 20;
    blobParams.minThreshold = 20;
    blobParams.maxThreshold = 230;
    blobParams.thresholdStep = 10;
    blobParams.minRepeatability = 2;

    blobParams.filterByColor = true;
    blobParams.blobColor = 255;

    blobParams.filterByArea = true;
    blobParams.minArea = 3000;
    blobParams.maxArea = 240000;

    blobParams.filterByInertia = false;
    blobParams.filterByConvexity = false;
    blobParams.filterByCircularity = false;

    SimpleBlobDetector blobDetector = SimpleBlobDetector(blobParams);
    std::vector<std::shared_ptr<TargetBoundaryInfo>> trackedTargets;

    // 1 second prune time threshold
    int64 targetPruneTime = (int64)(cv::getTickFrequency() * 1);

    int histogramChannels[] = { 0, 1 };
    float hueRange[] = { 0, 179 };
    float satRange[] = { 0, 255 };
    const float* histogramRanges[] = { hueRange, satRange };
    int numHueBins = 30;
    int numSatBins = 32;
    int histogramNumBins[] = { numHueBins, numSatBins };

    timers.printTableHeader();

    Mat sourceFrame, hsvFrame, threshFrame, backprojFrame, histogramRender;
    Mat targetColorHistogram;
    for(uint32_t frameNumber = 0; capture.isOpened(); frameNumber++)
    {
        timers.start();
        fpsCounter.addFrameTimestamp();

        capture >> sourceFrame;

        timers.markCheckpoint(CAPTURE_CHECKPOINT);

        //medianBlur(sourceFrame, sourceFrame, 5);
        cvtColor(sourceFrame, hsvFrame, CV_BGR2HSV);

        timers.markCheckpoint(CVT_COLOR_CHECKPOINT);

        if(enableThreshold)
            inRange(hsvFrame, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), threshFrame);

        timers.markCheckpoint(THRESH_CHECKPOINT);

        if (selectedTarget != nullptr && selectedTarget->area() > 0)
        {
            // TODO: Transition selection code to smart_ptr
            //printf("Updating target histogram with rect (x: %d, y: %d) (w: %d, h: %d)\r\n", selectedTarget->x, selectedTarget->y, selectedTarget->width, selectedTarget->height);
            Mat sourceRoi = Mat(hsvFrame, *selectedTarget);
            Mat maskRoi = threshFrame.empty() ? Mat() : Mat(threshFrame, *selectedTarget);

            calcHist(&sourceRoi, 1, histogramChannels, maskRoi, targetColorHistogram, includeSatInHist ? 2 : 1, histogramNumBins, histogramRanges);
            normalize(targetColorHistogram, targetColorHistogram, 0, 255, NORM_MINMAX);

            // TODO: Re-write histogram render for 2d histogram with sat
            //std::cout << targetColorHistogram << std::endl;

            if (!includeSatInHist)
            {
                histogramRender = Mat::zeros(200, numHueBins * 10, CV_8UC3);
                renderHueHistogram(targetColorHistogram, histogramRender, numHueBins);
                imshow("Histogram render", histogramRender);
            }

            delete selectedTarget;
            selectedTarget = nullptr;
        }


        if (!targetColorHistogram.empty() && (targetColorHistogram.cols == 1) == !includeSatInHist)
        {
            calcBackProject(&hsvFrame, 1, histogramChannels, targetColorHistogram, backprojFrame, histogramRanges);
            if(!threshFrame.empty())
                backprojFrame &= threshFrame;

            erode(backprojFrame, backprojFrame, Mat(), Point(-1, -1), 3);
            dilate(backprojFrame, backprojFrame, Mat(), Point(-1, -1), 3);

            timers.markCheckpoint(BACK_PROJECT_CHECKPOINT);

            if (frameNumber % 5 == 0)
            {
                std::vector<KeyPoint> blobKeyPoints;
                blobDetector.detect(backprojFrame, blobKeyPoints);
                updateTargetCorrelation(trackedTargets, blobKeyPoints, cv::getTickCount(), targetPruneTime);
            }

            timers.markCheckpoint(BLOB_DETECT_CHECKPOINT);

            for (auto trackedTarget : trackedTargets)
            {
                // TODO: Tune cam shift params
                if(trackedTarget->targetBounds->area() > 0)
                    trackedTarget->lastTrackedPose = CamShift(backprojFrame, *trackedTarget->targetBounds.get(), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 1));
            }

            timers.markCheckpoint(CAM_SHIFT_CHECKPOINT);

            if (!disableImshow)
            {
                for (auto trackedTarget : trackedTargets)
                {
                    if (trackedTarget->lastTrackedPose.size.area() > 0)
                        ellipse(sourceFrame, trackedTarget->lastTrackedPose, Scalar(255, 0, 255), 1);
                }
            }

            timers.markCheckpoint(RENDER_CHECKPOINT);

            if(!disableImshow)
                imshow("Backprojected frame", backprojFrame);
        }
        else
        {
            timers.markCheckpoint(BACK_PROJECT_CHECKPOINT);
            timers.markCheckpoint(BLOB_DETECT_CHECKPOINT);
            timers.markCheckpoint(CAM_SHIFT_CHECKPOINT);
            timers.markCheckpoint(RENDER_CHECKPOINT);
        }

        timers.stop();
        timers.printTableRow();

        printf(" \t(%.1f FPS)", fpsCounter.getFps());

        if (!disableImshow || targetColorHistogram.empty())
            imshow("Source", sourceFrame);

        if (!disableImshow && enableThreshold)
            imshow("Thresh", threshFrame);

        waitKey(1);
    }
}
