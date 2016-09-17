#include "RobotComms.h"

RobotComms::RobotComms(std::string targetServer, unsigned short targetPort)
{
    this->targetServer = targetServer;
    this->targetPort = targetPort;
}

void RobotComms::sendObject(JSON jsonObject, string packetType)
{
    JSON packetWrapperObject;
    packetWrapperObject["packetId"] = packetIndex++;
    packetWrapperObject["packetType"] = packetType;
    packetWrapperObject["packetPayload"] = jsonObject;

    string jsonDump = packetWrapperObject.dump();
    const char* bufPtr = jsonDump.c_str();
    int bufferLen = jsonDump.length();

    socket.sendTo(bufPtr, bufferLen, targetServer, targetPort);
}

void RobotComms::sendTrackedTargets(std::vector<std::shared_ptr<TargetBoundaryInfo>> targets)
{
    // TODO: include current time, frame number, etc
    // TODO: Include time for each item

    JSON packetData;
    packetData["trackedTargets"] = JSON::array();

    for (std::shared_ptr<TargetBoundaryInfo>& target : targets)
    {
        JSON targetObj;
        targetObj["poseRect"] =
        {
            { "center", { { "x", target->lastTrackedPose.center.x }, { "y", target->lastTrackedPose.center.y } } },
            { "width", target->lastTrackedPose.size.width },
            { "height", target->lastTrackedPose.size.height },
            { "angle", target->lastTrackedPose.angle }
        };

        targetObj["isTracked"] = target->isTracked;
        targetObj["xOffset"] = target->xOffset;
        targetObj["yOffset"] = target->yOffset;

        packetData["trackedTargets"].push_back(targetObj);
    }

    sendObject(packetData, "trackedTargetsSnapshot");
}
