#pragma once

#include "lib/PracticalSocket.h"
#include "lib/json.h"
#include "ColorBasedTargetDetector.h"
#include <string>
#include <memory>

using JSON = nlohmann::json;

class RobotComms
{
private:
    UDPSocket socket;
    std::string targetServer;
    unsigned short targetPort;

    uint64_t packetIndex = 0;

public:
    RobotComms(std::string targetServer, unsigned short targetPort);

    void sendObject(JSON jsonObject, string packetType);
    void sendTrackedTargets(std::vector<std::shared_ptr<TargetBoundaryInfo>> targets);
};
