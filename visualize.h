#pragma once

#include "ekf_slam.h"
#include "simulator.h"

class Visualizer {
public:
    void draw(const EKFSLAM& ekf, const Simulator& sim);
};
