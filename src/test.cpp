#include <iostream>
#include "Reader.hpp"
#include "State.hpp"
#include "MCL.hpp"

int main() {
    Reader r("/home/jason/Code/DSA/mcl-vs-kalman/robot_dataset.csv");

    State s;
    MCL mcl = MCL(12, 12, 6, 6, 500, 500, 1, 0.1, 0.5);
    mcl.spawnParticles();
    std::cout << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < 10000; i++) {
        s = r.bake_next_state();
        mcl.updateOdom(s.odom);
        mcl.updateSensors(s.sensors);
        mcl.propogateParticles();
        mcl.resampleParticles();

        Pose est = mcl.getEstimation();

        std::cout << "Delta:     (" << std::setw(8) << (s.truth.x - est.x) 
        << ", " << std::setw(8) << (s.truth.y - est.y) << ") | "
        << "True Values: (" << std::setw(8) << s.truth.x 
        << ", " << std::setw(8) << s.truth.y << ") | "
        << "Estimated Position: (" << std::setw(8) << est.x 
        << ", " << std::setw(8) << est.y << ") - " << mcl.getParticles().size() << std::endl;
    }
}