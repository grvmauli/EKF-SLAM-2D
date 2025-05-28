#include "ekf_slam.h"
#include "simulator.h"
#include "visualize.h"

int main() {
    EKFSLAM ekf(5);
    Simulator sim(5);
    Visualizer vis;

    for (int step = 0; step < 100; ++step) {
        ControlInput u = sim.getControl();
        std::vector<Observation> z = sim.getObservations();  // <- fixed here
        ekf.predict(u);
        ekf.update(z);
        vis.draw(ekf, sim);
    }
    return 0;
}
