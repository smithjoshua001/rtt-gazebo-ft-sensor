#include <rtt-gazebo-ft-sim.hpp>
#include <Eigen/Dense>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void ftSim::WorldUpdateBegin() {
    if (!is_configured && !isRunning())
        return;

    std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->getCommand();

    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->sense();
}

void ftSim::WorldUpdateEnd() {
    if (!is_configured && !isRunning())
        return;

    std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->move();
}
