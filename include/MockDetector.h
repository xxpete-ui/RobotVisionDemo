#pragma once

#include "IDetector.h"

#include <vector>

class MockDetector final : public IDetector
{
public:
	explicit MockDetector(
		std::vector<Target> targets);

	std::vector<Target> detect() override;

	void setTargets(
		std::vector<Target> targets);

private:
	std::vector<Target> targets_;
};