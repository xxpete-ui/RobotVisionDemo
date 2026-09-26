#pragma once

#include "VisionTypes.h"

#include <optional>
#include <string>

namespace CalibrationLoader
{
    std::optional<CameraConfig> load(
        const std::string& path,
        double demoDepthMeters);
}