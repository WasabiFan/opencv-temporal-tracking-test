#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>

struct TargetBoundaryInfo
{
    std::shared_ptr<cv::Rect> targetBounds;
    cv::RotatedRect lastTrackedPose;
    int64 lastDetectedTime = -1;
	bool isTracked = false;
};

enum TrackerMode
{
    TRACKER_NONE = 0,
    TRACKER_HUE,
    TRACKER_HUE_SAT
};

class ColorBasedTargetDetector
{
private:
    static const int histogramChannels[];
    static const float hueRange[];
    static const float satRange[];
    static const float* const histogramRanges[];
    static const int numHueBins;
    static const int numSatBins;
    static const int histogramNumBins[];

    int64_t targetPruneTimeThresh;
    uint16_t trackingUpdateInterval;
    uint64_t frameNum = 0;
	std::shared_ptr<TargetBoundaryInfo> selectedTarget = nullptr; 

    cv::Mat backprojFrameBuf;
    cv::Mat targetHistogram;

    // Use shared_ptr to make it easy to dynamically modify struct values
    std::vector<std::shared_ptr<TargetBoundaryInfo>> trackedTargets;
    cv::SimpleBlobDetector blobDetector;

    void updateTargetCorrelation(std::vector<cv::KeyPoint> detectedBlobs, int64_t currentTime);

public:

    struct Params
    {
        int blurSize = 31;
        int blurSigma = 15;
        int toZeroThresh = 10;
    };

    static void calculateHistFromTarget(cv::Mat& outHistogram, TrackerMode histMode, cv::Mat targetImage, cv::Mat targetMask = cv::Mat());

    ColorBasedTargetDetector(int64_t targetPruneTimeThresh, uint16_t trackingUpdateInterval);

    void updateTargetHistogram(cv::Mat newTargetHistogram);
    bool hasTargetTraining();

    void updateTracking(cv::Mat newFrame, int64_t currentTime, Params trackingParams = Params(), cv::Mat mask = cv::Mat());
    std::vector<std::shared_ptr<TargetBoundaryInfo>> getTrackedTargets();
    void getLastBackprojFrame(cv::Mat& outBackprojFrame);
	std::shared_ptr<TargetBoundaryInfo> selectTarget(std::shared_ptr<TargetBoundaryInfo> selectedTarget);

    ~ColorBasedTargetDetector();
};

