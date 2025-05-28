#include "visualize.h"
#include <iostream>

void Visualizer::draw(const EKFSLAM& ekf, const Simulator& sim) {
    auto state = ekf.getState();
    auto pose = sim.getTruePose();
    std::cout << "True Pose: (" << pose.x << ", " << pose.y << ", " << pose.theta << ")\n";
    std::cout << "Estimated: (" << state(0) << ", " << state(1) << ", " << state(2) << ")\n";
}
