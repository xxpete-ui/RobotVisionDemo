#pragma once

#include "VisionTypes.h"

#include <vector>

class IDetector
{
public:
	virtual ~IDetector() = default;

	virtual std::vector<Target> detect() = 0;
};