#pragma once
#include "utils.h"
#include <vector>

class Simulator {
public:
    Simulator(int num_landmarks);

    ControlInput getControl();
    std::vector<Observation> getObservations();

    Pose getTruePose() const;
    std::vector<Landmark> getLandmarks() const;

private:
    Pose pose;
    std::vector<Landmark> landmarks;
};