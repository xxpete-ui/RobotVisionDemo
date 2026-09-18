#include "RobotVision.h"
#include<iostream>
#include<vector>
#include<string>


bool testRecovery() {
    CameraConfig camera{ 2.0, 800.0, 800.0, 640.0, 360.0 };
    const double transform[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    RobotVision vision(camera, transform);
    std::vector<Target> targets{ {3, 0.80, 720, 400, true} };
    ValidTarget result{};

    if (vision.run(targets, result) ||
        vision.getStatus() != VisionStatus::NoValidTarget) {
        std::cerr << "FAIL: expected NoValidTarget\n";
        return false;
    }

    targets[0].grabbed = false;
    if (!vision.run(targets, result) ||
        vision.getStatus() != VisionStatus::OK ||
        result.target.id != 3) {
        std::cerr << "FAIL: expected recovery with target ID 3\n";
        return false;
    }

    return true;
}


bool testGrabbedTargetFiltering() {
    CameraConfig camera{ 2.0, 800.0, 800.0, 640.0, 360.0 };
    const double transform[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    RobotVision vision(camera, transform);
    std::vector<Target> targets{
        {1, 0.72, 700, 380, false},
        {2, 0.75, 720, 400, false}
    };
    ValidTarget selected{};

    if (markTargetGrabbed(targets, 999) ||
        targets[0].grabbed || targets[1].grabbed) {
        std::cerr << "FAIL: unknown ID changed target state\n";
        return false;
    }

    if (!vision.run(targets, selected) || selected.target.id != 2) {
        std::cerr << "FAIL: expected target ID 2 first\n";
        return false;
    }

    if (!markTargetGrabbed(targets, selected.target.id) ||
        !targets[1].grabbed) {
        std::cerr << "FAIL: could not mark target ID 2\n";
        return false;
    }

    if (!vision.run(targets, selected) || selected.target.id != 1) {
        std::cerr << "FAIL: expected target ID 1 after grabbing ID 2\n";
        return false;
    }

    return true;
}


int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: RobotVisionStateTest recovery|filtering\n";
        return 2;
    }

    const std::string scenario = argv[1];

    if (scenario == "recovery") {
        return testRecovery() ? 0 : 1;
    }
    if (scenario == "filtering") {
        return testGrabbedTargetFiltering() ? 0 : 1;
    }

    std::cerr << "Unknown scenario: " << scenario << '\n';
    return 2;
}