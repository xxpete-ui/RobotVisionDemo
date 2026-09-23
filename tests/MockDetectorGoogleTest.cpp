#include "MockDetector.h"

#include <gtest/gtest.h>

#include <vector>

TEST(
	MockDetectorTest,
	ReturnsConfiguredTargets)
{
	const std::vector<Target> configuredTargets{
		{1, 0.90, 720.0, 400.0, false},
		{2, 0.75, 640.0, 360.0, true}
	};

	MockDetector detector(configuredTargets);

	const std::vector<Target> detectedTargets =
		detector.detect();

	ASSERT_EQ(detectedTargets.size(), 2U);

	EXPECT_EQ(detectedTargets[0].id, 1);
	EXPECT_DOUBLE_EQ(detectedTargets[0].confidence, 0.90);
	EXPECT_DOUBLE_EQ(detectedTargets[0].x, 720.0);
	EXPECT_DOUBLE_EQ(detectedTargets[0].y, 400.0);
	EXPECT_FALSE(detectedTargets[0].grabbed);

	EXPECT_EQ(detectedTargets[1].id, 2);
	EXPECT_DOUBLE_EQ(detectedTargets[1].confidence, 0.75);
	EXPECT_TRUE(detectedTargets[1].grabbed);
}

TEST(
    MockDetectorTest,
    ReturnsEmptyTargetsWhenConfiguredEmpty)
{
    MockDetector detector({});

    const std::vector<Target> detectedTargets =
        detector.detect();

    EXPECT_TRUE(detectedTargets.empty());
}

TEST(
    MockDetectorTest,
    ReturnsIndependentResults)
{
    const std::vector<Target> configuredTargets{
        {7, 0.85, 720.0, 400.0, false}
    };

    MockDetector detector(configuredTargets);

    std::vector<Target> firstResult =
        detector.detect();

    ASSERT_EQ(firstResult.size(), 1U);

    firstResult[0].confidence = 0.0;
    firstResult[0].grabbed = true;

    const std::vector<Target> secondResult =
        detector.detect();

    ASSERT_EQ(secondResult.size(), 1U);

    EXPECT_DOUBLE_EQ(
        secondResult[0].confidence,
        0.85);

    EXPECT_FALSE(
        secondResult[0].grabbed);
}

TEST(
    MockDetectorTest,
    UpdatesConfiguredTargets)
{
    MockDetector detector({
        {1, 0.70, 680.0, 360.0, false}
        });

    const std::vector<Target> firstResult =
        detector.detect();

    ASSERT_EQ(firstResult.size(), 1U);
    EXPECT_EQ(firstResult[0].id, 1);

    detector.setTargets({
        {2, 0.90, 720.0, 400.0, false},
        {3, 0.80, 600.0, 350.0, false}
        });

    const std::vector<Target> secondResult =
        detector.detect();

    ASSERT_EQ(secondResult.size(), 2U);

    EXPECT_EQ(secondResult[0].id, 2);
    EXPECT_EQ(secondResult[1].id, 3);

}