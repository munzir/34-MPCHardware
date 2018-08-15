//
// Created by Shimin Zhang on 8/14/18.
//

#include "helpers.h"
#include "kore/display.hpp"
#include "file_ops.hpp"
#include "utils.h"
#include "mpc_ddp.h"

using namespace std;
using namespace Krang;

extern struct MPC_Config mpcConfig;

int main () {
    cout << "MPC Config Final Time Before Load" << mpcConfig.finalTime << endl;
    readMDPConfig();
    cout << "MPC Config Final Time After Load" << mpcConfig.finalTime << endl;
    cout << "MPC Config Goal State" << mpcConfig.goalState << endl;

    return 0;
}



