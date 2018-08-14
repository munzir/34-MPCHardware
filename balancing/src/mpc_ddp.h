//
// Created by Sherlock Hummus on 8/13/18.
//

#include "krang_dynamics.h"

#ifndef DEB8_DEMOS_MPC_DDP_H
#define DEB8_DEMOS_MPC_DDP_H

// Get MPC DDP configure info
void readMDPConfig();

// Compute Initial DDP trajectory
void computeDDPTrajectory();

// Compute Initial DDP trajectory
void mpcTrajUpdate();

// mpc ddp thread
void *mpcddp(void *);
#endif //DEB8_DEMOS_MPC_DDP_H
