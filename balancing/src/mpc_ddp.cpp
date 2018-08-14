//
// Created by Shimin Zhang on 8/13/18.
//

#include "helpers.h"
#include "mpc_ddp.h"

using namspace config4cpp;
using namspace std;

/* ******************************************************************************************** */
// Read and Configure DDP Parameters
void readMDPConfig(MPC_Config &config) {
    Configuration *  cfg = Configuration::create();
    const char *     scope = "";
    const char *     configFile = "../src/controlParams.cfg";
    const char * str;
    std::istringstream stream;

    try {
        cfg->parse(configFile);

        str = cfg->lookupString(scope, "goalState");
        stream.str(str); for(int i=0; i<8; i++) stream >> config.goalState(i); stream.clear();

        config.finalTime = cfg->lookupFloat(scope, "finalTime");

        config.DDPMaxIter = cfg->lookupInt(scope, "DDPMaxIter");

        str = cfg->lookupString(scope, "DDPStatePenalties");
        stream.str(str); for(int i=0; i<8; i++) stream >> config.DDPStatePenalties(i); stream.clear();

        str = cfg->lookupString(scope, "DDPTerminalStatePenalties");
        stream.str(str); for(int i=0; i<8; i++) stream >> config.DDPTerminalStatePenalties(i); stream.clear();

        str = cfg->lookupString(scope, "DDPControlPenalties");
        stream.str(str); for(int i=0; i<2; i++) stream >> config.DDPControlPenalties(i); stream.clear();

        config.beginStep = cfg->lookupInt(scope, "beginStep");

        config.MPCMaxIter = cfg->lookupInt(scope, "MPCMaxIter");

        config.MPCHorizon = cfg->lookupInt(scope, "MPCHorizon");

        str = cfg->lookupString(scope, "MPCStatePenalties");
        stream.str(str); for(int i=0; i<8; i++) stream >> config.MPCStatePenalties(i); stream.clear();

        str = cfg->lookupString(scope, "MPCTerminalStatePenalties");
        stream.str(str); for(int i=0; i<8; i++) stream >> config.MPCTerminalStatePenalties(i); stream.clear();

        str = cfg->lookupString(scope, "MPCControlPenalties");
        stream.str(str); for(int i=0; i<2; i++) stream >> config.MPCControlPenalties(i); stream.clear();

        str = cfg->lookupString(scope, "tauLim");
        stream.str(str); for(int i=0; i<18; i++) stream >> config.tauLim(i); stream.clear();

    } catch(const ConfigurationException & ex) {
        cerr << ex.c_str() << endl;
        cfg->destroy();
    }
}
/* ******************************************************************************************** */
// Compute Initial DDP Trajectory
void computeDDPTraj() {

}

/* ******************************************************************************************** */
// Run MPC-DDP and Update Trajectory
void mpcTrajUpdate() {

}

/* ******************************************************************************************** */
// Thread for MPC DDP Related functionalities
void *mpcddp(void *) {

}

