#pragma once

struct Pose {
    double x, y, theta;
};

struct ControlInput {
    double v; // linear velocity
    double w; // angular velocity
};

struct Observation {
    int id;
    double range;
    double bearing;
};

struct Landmark {
    int id;
    double x;
    double y;
};
