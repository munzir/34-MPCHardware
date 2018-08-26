//
// Created by Shimin Zhang on 8/13/18.
//

#include "krang_dynamics.h"
#include <ddp/costs.hpp>
#include <ddp/ddp.hpp>
#include <ddp/mpc.hpp>
#include <ddp/util.hpp>
#include <config4cpp/Configuration.h>

#ifndef DEB8_DEMOS_MPC_DDP_H
#define DEB8_DEMOS_MPC_DDP_H

// alias for DDP variables
using Scalar = double;
using DDPDynamics = Krang3D<Scalar>;
using DDP_Opt = optimizer::DDP<DDPDynamics>;
using Cost = Krang3DCost<Scalar>;
using TerminalCost = Krang3DTerminalCost<Scalar>;
using StateTrajectory = typename DDPDynamics::StateTrajectory ;
using ControlTrajectory= typename DDPDynamics::ControlTrajectory ;
using State = typename DDPDynamics::State;
using Control = typename DDPDynamics::Control;

// Config Struct
struct MPC_Config {
    State goalState;
    double finalTime;
    int DDPMaxIter;
    Eigen::Matrix<double, 8, 1> DDPStatePenalties;
    Eigen::Matrix<double, 8, 1> DDPTerminalStatePenalties;
    Eigen::Matrix<double, 2, 1> DDPControlPenalties;
    int beginStep;
    int MPCMaxIter;
    int MPCHorizon;
    float MPCdt;
    Eigen::Matrix<double, 8, 1> MPCStatePenalties;
    Eigen::Matrix<double, 8, 1> MPCTerminalStatePenalties;
    Eigen::Matrix<double, 2, 1> MPCControlPenalties;
    Eigen::Matrix<double, 18, 1> tauLim;
};


struct DDP_Result {
    StateTrajectory stateTraj;
    ControlTrajectory controlTraj;
};

extern struct MPC_Config g_mpcConfig;
extern struct DDP_Result g_ddpResult;

// mutex for ddp related global states info
extern Vector6d g_state;
extern pthread_mutex_t g_state_mutex;
extern Vector2d g_augstate;
extern pthread_mutex_t g_augstate_mutex;
extern pthread_mutex_t g_robot_mutex;
extern double g_xInit, g_psiInit;
extern pthread_mutex_t MODE_mutex;

extern double g_mpc_init_time;
extern pthread_mutex_t g_mpc_init_time_mutex;

extern ControlTrajectory g_mpc_trajectory_main;
extern ControlTrajectory g_mpc_trajectory_backup;
extern pthread_mutex_t g_mpc_trajectory_main_mutex;
extern pthread_mutex_t g_mpc_trajectory_backup_mutex;


///* ******************************************************************************************* */
////Parameters for DDP
//ControlTrajectory mDDPControlTraj;
//StateTrajectory mDDPStateTraj;
//DDPDynamics *mDDPDynamics;
//Control mMPCControlRef;  // Store all the reference trajectory MPC compute
//Control u;  // Actual Control that is being given
//State mMPCStateRef;
//int mSteps;
//int mMPCSteps;
//double mMPCdt;
//double mR = 0.25;
//double mL = 0.68;
//CSV_writer<Scalar> mMPCWriter;
//
//double time_ddp;
//double timeddp_previous = 0;


// create simplified 3dof simulation
SkeletonPtr create3DOF_URDF();

// Updates 3dof model with full krang model params
void getSimple(SkeletonPtr& threeDOF);

// Get MPC DDP configure info
void readMDPConfig();

// Create dynamics object baed on a particular three dof config
DDPDynamics* getDynamics(SkeletonPtr& threeDOF);

// Compute Initial DDP trajectory
void computeDDPTrajectory(SkeletonPtr& threeDof);

// Compute Initial DDP trajectory
void mpcTrajUpdate(SkeletonPtr& threeDOF);

// get current time
double get_time();

// update mpc initialization time
void update_mpc_time();

// return mpc initialization time
double get_mpc_init_time();

// Initialize MPC DDP
void initializeMPCDDP();

// Exit from MPCMode and return to balance low
void exitMPC();

// mpc ddp thread
void *mpcddp(void *);
#endif //DEB8_DEMOS_MPC_DDP_H
