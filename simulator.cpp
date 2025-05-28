#include "simulator.h"
#include <cmath>
#include <random>

Simulator::Simulator(int num_landmarks) {
    pose = {0, 0, 0};
    for (int i = 0; i < num_landmarks; ++i) {
        landmarks.push_back({i, 5.0 * cos(i), 5.0 * sin(i)});
    }
}

ControlInput Simulator::getControl() {
    pose.x += 0.1 * cos(pose.theta);
    pose.y += 0.1 * sin(pose.theta);
    pose.theta += 0.05;
    return {0.1, 0.05};
}

std::vector<Observation> Simulator::getObservations() {
    std::vector<Observation> obs;
    for (const auto& l : landmarks) {
        double dx = l.x - pose.x;
        double dy = l.y - pose.y;
        double range = sqrt(dx*dx + dy*dy);
        double bearing = atan2(dy, dx) - pose.theta;
        obs.push_back({l.id, range, bearing});
    }
    return obs;
}

Pose Simulator::getTruePose() const {
    return pose;
}

std::vector<Landmark> Simulator::getLandmarks() const {
    return landmarks;
}
