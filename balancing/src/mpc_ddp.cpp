//
// Created by Shimin Zhang on 8/13/18.
//

#include "helpers.h"
#include "mpc_ddp.h"

using namespace config4cpp;
using namespace std;

// Globals
MPC_Config mpcConfig;

/* ******************************************************************************************** */
// Read and Configure DDP Parameters
void readMDPConfig() {
    Configuration *  cfg = Configuration::create();
    const char *     scope = "";
    const char *     configFile = "../src/controlParams.cfg";
    const char * str;
    std::istringstream stream;

    try {
        cfg->parse(configFile);

        str = cfg->lookupString(scope, "goalState");
        stream.str(str); for(int i=0; i<8; i++) stream >> mpcConfig.goalState(i); stream.clear();

        mpcConfig.finalTime = cfg->lookupFloat(scope, "finalTime");

        mpcConfig.DDPMaxIter = cfg->lookupInt(scope, "DDPMaxIter");

        str = cfg->lookupString(scope, "DDPStatePenalties");
        stream.str(str); for(int i=0; i<8; i++) stream >> mpcConfig.DDPStatePenalties(i); stream.clear();

        str = cfg->lookupString(scope, "DDPTerminalStatePenalties");
        stream.str(str); for(int i=0; i<8; i++) stream >> mpcConfig.DDPTerminalStatePenalties(i); stream.clear();

        str = cfg->lookupString(scope, "DDPControlPenalties");
        stream.str(str); for(int i=0; i<2; i++) stream >> mpcConfig.DDPControlPenalties(i); stream.clear();

        mpcConfig.beginStep = cfg->lookupInt(scope, "beginStep");

        mpcConfig.MPCMaxIter = cfg->lookupInt(scope, "MPCMaxIter");

        mpcConfig.MPCHorizon = cfg->lookupInt(scope, "MPCHorizon");

        str = cfg->lookupString(scope, "MPCStatePenalties");
        stream.str(str); for(int i=0; i<8; i++) stream >> mpcConfig.MPCStatePenalties(i); stream.clear();

        str = cfg->lookupString(scope, "MPCTerminalStatePenalties");
        stream.str(str); for(int i=0; i<8; i++) stream >> mpcConfig.MPCTerminalStatePenalties(i); stream.clear();

        str = cfg->lookupString(scope, "MPCControlPenalties");
        stream.str(str); for(int i=0; i<2; i++) stream >> mpcConfig.MPCControlPenalties(i); stream.clear();

        str = cfg->lookupString(scope, "tauLim");
        stream.str(str); for(int i=0; i<18; i++) stream >> mpcConfig.tauLim(i); stream.clear();

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
    bool ddp_init = false;
    bool ddp_init_last = false;
    while (true) {
        // update our ddp init with global
        pthread_mutex_lock(&ddp_initialized_mutex);
            ddp_init = ddp_initialized;
        pthread_mutex_unlock(&ddp_initialized_mutex);

        // initialize ddp traj calculation if newly initialized
        if (ddp_init && !ddp_init_last) {
            computeDDPTraj();
            pthread_mutex_lock(&ddp_traj_rdy_mutex);
                ddp_traj_rdy = true;
            pthread_mutex_unlock(&ddp_traj_rdy_mutex);
        }

        // Update our track variables
        ddp_init_last = ddp_init;
    }
}

