#include "ColorBasedTargetDetector.h"

/*
NOTES:

- Dilate and erode dramatically decrease blob time
- Can do blobs every few frames to minimize slowdown
- Blur removed to speed up frame time. Can be added back if desired.
*/

bool operator==(cv::KeyPoint const& lhs, cv::KeyPoint const& rhs)
{
    return lhs.pt == rhs.pt
        && lhs.angle == rhs.angle
        && lhs.size == rhs.size
        && lhs.response == rhs.response
        && lhs.octave == rhs.octave
        && lhs.class_id == rhs.class_id;
}

const int ColorBasedTargetDetector::histogramChannels[] = { 0, 1 };
const float ColorBasedTargetDetector::hueRange[] = { 0, 179 };
const float ColorBasedTargetDetector::satRange[] = { 0, 255 };
const float* const ColorBasedTargetDetector::histogramRanges[] = { hueRange, satRange };
const int ColorBasedTargetDetector::numHueBins = 30;
const int ColorBasedTargetDetector::numSatBins = 16;
const int ColorBasedTargetDetector::histogramNumBins[] = { numHueBins, numSatBins };

void ColorBasedTargetDetector::updateTargetCorrelation(std::vector<cv::KeyPoint> detectedBlobs, int64_t currentTime)
{
    //TODO: Add logging
    std::vector<cv::KeyPoint> unpairedBlobs = detectedBlobs;
    std::vector<std::shared_ptr<TargetBoundaryInfo>> unpairedTargets = trackedTargets;
    for (cv::KeyPoint blob : detectedBlobs)
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
            trackedTargets.erase(std::remove(trackedTargets.begin(), trackedTargets.end(), unpairedTarget), trackedTargets.end());
    }

    for (cv::KeyPoint unpairedBlob : unpairedBlobs)
    {
        auto newTarget = std::make_shared<TargetBoundaryInfo>();
        newTarget->lastDetectedTime = currentTime;
        newTarget->targetBounds = std::make_shared<cv::Rect>();
        newTarget->targetBounds->x = (int)unpairedBlob.pt.x;
        newTarget->targetBounds->y = (int)unpairedBlob.pt.y;
        newTarget->targetBounds->width = (int)unpairedBlob.size;
        newTarget->targetBounds->height = (int)unpairedBlob.size;

        trackedTargets.push_back(newTarget);
    }

	if (trackedTargets.size() > maxTargets)
		trackedTargets.clear();
}

void ColorBasedTargetDetector::calculateHistFromTarget(cv::Mat& outHistogram, TrackerMode histMode, cv::Mat targetImage, cv::Mat targetMask)
{
    if (histMode == TRACKER_NONE)
    {
        throw std::exception("Cannot calculate tracker histogram without a specified mode!");
    }

    // TODO: I'm pretty sure that casting a const to a non-const is bad
    cv::calcHist(&targetImage, 1, (int*)histogramChannels, targetMask, outHistogram, (histMode == TRACKER_HUE_SAT) ? 2 : 1, (int*)histogramNumBins, (const float**)histogramRanges);
    cv::normalize(outHistogram, outHistogram, 0, 255, cv::NORM_MINMAX);
}

ColorBasedTargetDetector::ColorBasedTargetDetector(int64_t targetPruneTimeThresh, uint16_t trackingUpdateInterval)
{
    this->targetPruneTimeThresh = targetPruneTimeThresh;
    this->trackingUpdateInterval = trackingUpdateInterval;

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

    blobDetector = cv::SimpleBlobDetector(blobParams);
}

void ColorBasedTargetDetector::updateTargetHistogram(cv::Mat newTargetHistogram)
{
    this->targetHistogram = newTargetHistogram;
}

bool ColorBasedTargetDetector::hasTargetTraining()
{
    return !targetHistogram.empty();
}

void ColorBasedTargetDetector::updateTracking(cv::Mat newFrame, int64_t currentTime, Params trackingParams, cv::Mat mask)
{
    if (targetHistogram.empty())
        throw std::exception("A histogram must be calculated and loaded before tracking can be executed.");

    cv::calcBackProject(&newFrame, 1, histogramChannels, targetHistogram, backprojFrameBuf, (const float**)histogramRanges);
    if (!mask.empty())
        backprojFrameBuf &= mask;

    //cv::erode(backprojFrameBuf, backprojFrameBuf, cv::Mat(), cv::Point(-1, -1), 3);
    //cv::dilate(backprojFrameBuf, backprojFrameBuf, cv::Mat(), cv::Point(-1, -1), 3);
    int oddSize = trackingParams.blurSize | 1;
    cv::GaussianBlur(backprojFrameBuf, backprojFrameBuf, cv::Size(oddSize, oddSize), trackingParams.blurSigma);
    cv::threshold(backprojFrameBuf, backprojFrameBuf, trackingParams.toZeroThresh, 0, CV_THRESH_TOZERO);

    if (frameNum % trackingUpdateInterval == 0)
    {
        std::vector<cv::KeyPoint> blobKeyPoints;
        blobDetector.detect(backprojFrameBuf, blobKeyPoints);
        updateTargetCorrelation(blobKeyPoints, currentTime);
    }

    for (auto trackedTarget : trackedTargets)
    {
        // TODO: Tune cam shift params
        if (trackedTarget->targetBounds->area() > 0)
            trackedTarget->lastTrackedPose = CamShift(backprojFrameBuf, *trackedTarget->targetBounds.get(),
                cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 1));
    }
	
	selectedTarget = selectTarget(selectedTarget);
}

std::vector<std::shared_ptr<TargetBoundaryInfo>> ColorBasedTargetDetector::getTrackedTargets()
{
    return trackedTargets;
}

void ColorBasedTargetDetector::getLastBackprojFrame(cv::Mat& outBackprojFrame)
{
    outBackprojFrame = backprojFrameBuf;
}

std::shared_ptr<TargetBoundaryInfo> ColorBasedTargetDetector::selectTarget(std::shared_ptr<TargetBoundaryInfo> selectedTarget)
{
	std::shared_ptr<TargetBoundaryInfo> largestTarget = nullptr;
	
	if(trackedTargets.size() == 0)
		return nullptr;

	for (int i = 0; i < trackedTargets.size(); ++i)
	{
		if (selectedTarget == trackedTargets[i])
			return selectedTarget;

		if (largestTarget == nullptr)
			largestTarget = trackedTargets[i];
		else if (largestTarget->targetBounds->area() < trackedTargets[i]->targetBounds->area())
			largestTarget = trackedTargets[i];
	}

	largestTarget->isTracked = true;
	return largestTarget;
}

ColorBasedTargetDetector::~ColorBasedTargetDetector()
{
}
