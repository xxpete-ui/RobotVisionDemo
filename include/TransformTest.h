#pragma once
#include "VisionTypes.h"

bool testTransform();

void printRotationComposition();

void printRotateRobotPoint(const CameraPoint& cameraPoint);

bool demoLetterboxToCamera(const CameraConfig& config, CameraPoint& result);