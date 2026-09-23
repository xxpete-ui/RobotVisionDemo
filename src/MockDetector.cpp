#include "MockDetector.h"

#include <utility>

MockDetector::MockDetector(
    std::vector<Target> targets)
    : targets_(std::move(targets))
{
}

std::vector<Target> MockDetector::detect()
{
    return targets_;
}

void MockDetector::setTargets(
    std::vector<Target> targets)
{
    targets_ = std::move(targets);
}