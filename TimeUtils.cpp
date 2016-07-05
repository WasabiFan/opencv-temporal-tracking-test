#include "TimeUtils.h"

#include <iostream>
#include <stdio.h>
#include <map>
#include <opencv2/opencv.hpp>

TimerManager::TimerManager()
{
    addCheckpoint(TOTAL_CHECKPOINT_NAME);
}

TimerManager::~TimerManager()
{

}

void TimerManager::addCheckpoint(std::string name, std::string relativeTo)
{
    TimerCheckpoint newCheckpoint = { name, checkpoints.count(relativeTo) == 0 ? nullptr : &checkpoints.at(relativeTo), -1 };
    checkpoints[name] = newCheckpoint;
    orderedNames.push_back(name);
}

void TimerManager::printTableHeader()
{
    for (std::string name : orderedNames)
    {
        printf("%s ", name.c_str());
    }

    printf("\r\n");
}

void TimerManager::start()
{
    startTime = cv::getTickCount();
    for (std::string name : orderedNames)
    {
        checkpoints[name].endTime = -1;
    }
}

void TimerManager::markCheckpoint(std::string name)
{
    checkpoints[name].endTime = cv::getTickCount();
}

void TimerManager::stop()
{
    int64 endTime = cv::getTickCount();
    for (std::string name : orderedNames)
    {
        if (checkpoints[name].endTime < 0)
            checkpoints[name].endTime = endTime;
    }
}

void TimerManager::printTableRow(bool asOverwrite)
{
    if (asOverwrite)
        printf("\r");

    for (std::string name : orderedNames)
    {
        TimerCheckpoint checkpointData = checkpoints[name];
        int64 localStartTime = checkpointData.relativeTo == nullptr ? startTime : checkpointData.relativeTo->endTime;
        double totalTime = (checkpointData.endTime - localStartTime) / ((double) cvGetTickFrequency()*1000.);

        printf("%*d ", name.size(), (int)totalTime);
    }

    if (!asOverwrite)
        printf("\r\n");
}