//
// Created by Shimin Zhang on 8/13/18.
//

#include "helpers.h"
#include "mpc_ddp.h"
#include "krang_dynamics.h"

using namespace config4cpp;
using namespace std;

// Globals
MPC_Config mpcConfig;
DDP_Result ddp_result;


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

        mpcConfig.MPCdt = cfg->lookupFloat(scope, "MPCdt");

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
// Create simplified 3 dof simulation for high level controller
SkeletonPtr create3DOF_URDF()
{
    dart::utils::DartLoader loader;
    SkeletonPtr threeDOF =
            loader.parseSkeleton("/home/shimin/krang/09-URDF/3DOF-WIP/3dof.urdf");
    threeDOF->setName("m3DOF");

    threeDOF->getJoint(0)->setDampingCoefficient(0, 0.5);
    threeDOF->getJoint(1)->setDampingCoefficient(0, 0.5);

    return threeDOF;
}

/* ******************************************************************************************** */
// Update the position and velocity of a 3dof dart model with that of the global krang model
void updateSimple(SkeletonPtr& threeDOF) {
    pthread_mutex_lock(&g_robot_mutex);
    // Body Mass
    double mFull =g_robot->getMass();
    double mLWheel = g_robot->getBodyNode("LWheel")->getMass();
    double mRWheel = g_robot->getBodyNode("RWheel")->getMass();
    double mBody = mFull - mLWheel - mRWheel;

    Eigen::Vector3d bodyCOM;
    dart::dynamics::Frame* baseFrame = g_robot->getBodyNode("Base");
    bodyCOM = (mFull*g_robot->getCOM(baseFrame) - mLWheel*g_robot->getBodyNode("LWheel")->getCOM(baseFrame) - mLWheel*g_robot->getBodyNode("RWheel")->getCOM(baseFrame))/(mFull - mLWheel - mRWheel);

    // Body inertia (axis)
    double m;
    Eigen::Matrix3d iMat;
    Eigen::Matrix3d iBody = Eigen::Matrix3d::Zero();
    double ixx, iyy, izz, ixy, ixz, iyz;
    Eigen::Matrix3d rot;
    Eigen::Vector3d t;
    Eigen::Matrix3d tMat;
    dart::dynamics::BodyNodePtr b;
    int nBodies = g_robot->getNumBodyNodes();

    for(int i=0; i<nBodies; i++){
        if(i==1 || i==2) continue; // Skip wheels
        b = g_robot->getBodyNode(i);
        b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
        rot = b->getTransform(baseFrame).rotation();
        t = g_robot->getCOM(baseFrame) - b->getCOM(baseFrame) ; // Position vector from local COM to body COM expressed in base frame
        m = b->getMass();
        iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
                ixy, iyy, iyz,
                ixz, iyz, izz;
        iMat = rot*iMat*rot.transpose(); // Inertia tensor of the body around its CoM expressed in base frame
        tMat << (t(1)*t(1)+t(2)*t(2)), (-t(0)*t(1)),          (-t(0)*t(2)),
                (-t(0)*t(1)),          (t(0)*t(0)+t(2)*t(2)), (-t(1)*t(2)),
                (-t(0)*t(2)),          (-t(1)*t(2)),          (t(0)*t(0)+t(1)*t(1));
        iMat = iMat + m*tMat; // Parallel Axis Theorem
        iBody += iMat;
    }

    // Aligning threeDOF base frame to have the y-axis pass through the CoM
    double th = atan2(bodyCOM(2), bodyCOM(1));
    rot << 1, 0, 0,
            0, cos(th), sin(th),
            0, -sin(th), cos(th);
    bodyCOM = rot*bodyCOM;
    iBody = rot*iBody*rot.transpose();

    // Set the 3 DOF robot parameters
    threeDOF->getBodyNode("Base")->setMomentOfInertia(iBody(0,0), iBody(1,1), iBody(2,2), iBody(0,1), iBody(0,2), iBody(1,2));
    threeDOF->getBodyNode("Base")->setLocalCOM(bodyCOM);
    threeDOF->getBodyNode("Base")->setMass(mBody);

    // Update 3DOF state
    // get positions
    Eigen::Matrix3d baseRot = g_robot->getBodyNode("Base")->getTransform().rotation();
    baseRot = baseRot*rot.transpose();
    Eigen::AngleAxisd aa(baseRot);
    Eigen::Matrix<double, 8, 1> q, dq;

    q << aa.angle()*aa.axis(), g_robot->getPositions().segment(3, 5);
    threeDOF->setPositions(q);
    // TODO: When joints are unlocked qBody1 of the 3DOF (= dth = COM angular speed) is not the same as qBody1 of the full robot
    dq << rot*g_robot->getVelocities().head(3), rot*g_robot->getVelocities().segment(3, 3), g_robot->getVelocities().segment(6, 2);
    threeDOF->setVelocities(dq);
    pthread_mutex_unlock(&g_robot_mutex);
}


/* ******************************************************************************************** */
// Compute Initial DDP Trajectory
void computeDDPTrajectory(SkeletonPtr& threeDOF) {
    // TODO Figure out where this belongs
    double mR = 0.25;
    double mL = 0.68;//*6;
    // TODO END
    param p;
    double ixx, iyy, izz, ixy, ixz, iyz;
    Eigen::Vector3d com;
    Eigen::Matrix3d iMat;
    Eigen::Matrix3d tMat;

    CSV_writer<Scalar> writer;
    util::DefaultLogger logger;
    bool verbose = true;

    Scalar tf = mpcConfig.finalTime;
    auto time_steps = util::time_steps(tf, mpcConfig.MPCdt);

    DDPDynamics::ControlTrajectory u = DDPDynamics::ControlTrajectory::Zero(2, time_steps);

    // Update params with 3dof simulation
    dart::dynamics::Frame* baseFrame = threeDOF->getBodyNode("Base");
    p.R = mR; p.L = mL; p.g=9.800000e+00;

    p.mw =threeDOF->getBodyNode("LWheel")->getMass();

    threeDOF->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);

    p.YYw = ixx; p.ZZw = izz; p.XXw = iyy; // Wheel frame of reference in ddp dynamic model is different from the one in DART
    p.m_1 = threeDOF->getBodyNode("Base")->getMass();
    com = threeDOF->getBodyNode("Base")->getCOM(baseFrame);
    p.MX_1 = p.m_1*com(0); p.MY_1 = p.m_1*com(1); p.MZ_1 = p.m_1*com(2);

    threeDOF->getBodyNode("Base")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    Eigen::Vector3d s = -com; // Position vector from local COM to body COM expressed in base frame
    iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
            ixy, iyy, iyz,
            ixz, iyz, izz;
    tMat << (s(1)*s(1)+s(2)*s(2)), (-s(0)*s(1)),          (-s(0)*s(2)),
            (-s(0)*s(1)),          (s(0)*s(0)+s(2)*s(2)), (-s(1)*s(2)),
            (-s(0)*s(2)),          (-s(1)*s(2)),          (s(0)*s(0)+s(1)*s(1));
    iMat = iMat + p.m_1*tMat; // Parallel Axis Theorem
    p.XX_1 = iMat(0,0); p.YY_1 = iMat(1,1); p.ZZ_1 = iMat(2,2);
    p.XY_1 = iMat(0,1); p.YZ_1 = iMat(1,2); p.XZ_1 = iMat(0,2);
    p.fric_1 = threeDOF->getJoint(0)->getDampingCoefficient(0); // Assuming both joints have same friction coeff (Please make sure that is true)

    // initialize Dynamics
    DDPDynamics* opt_dynamics = new DDPDynamics(p);

    //Lock then initialize the state with the current state of Krang
    pthread_mutex_lock(&g_state_mutex);
    pthread_mutex_lock(&g_augstate_mutex);
    State x0; x0 << g_state(2),g_state(4),g_state(0),g_state(3),g_state(5),g_state(1),g_augstate(0), g_augstate(1);
    pthread_mutex_unlock(&g_state_mutex);
    pthread_mutex_unlock(&g_augstate_mutex);

    cout << "initState: " << x0.transpose() << endl;

    // Update Costs
    Cost::StateHessian Q;
    Q.setZero();
    Q.diagonal() << mpcConfig.DDPStatePenalties;

    Cost::ControlHessian R;
    R.setZero();
    R.diagonal() << mpcConfig.DDPControlPenalties;

    TerminalCost::Hessian Qf;
    Qf.setZero();
    Qf.diagonal() << mpcConfig.DDPTerminalStatePenalties;

    Cost cp_cost(mpcConfig.goalState, Q, R);
    TerminalCost cp_terminal_cost(mpcConfig.goalState, Qf);

    // initialize DDP for trajectory planning
    DDP_Opt trej_ddp (mpcConfig.MPCdt, time_steps, mpcConfig.DDPMaxIter, &logger, verbose);

    // Get initial trajectory from DDP
    OptimizerResult<DDPDynamics> DDP_traj = trej_ddp.run(x0, u, *opt_dynamics, cp_cost, cp_terminal_cost);

    ddp_result.stateTraj = DDP_traj.state_trajectory;
    ddp_result.controlTraj = DDP_traj.control_trajectory;

    writer.save_trajectory(ddp_result.stateTraj, ddp_result.controlTraj, "initial_traj.csv");
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
    SkeletonPtr threeDOF = create3DOF_URDF();
    while (true) {
        // update our ddp init with global
        pthread_mutex_lock(&ddp_initialized_mutex);
            ddp_init = ddp_initialized;
        pthread_mutex_unlock(&ddp_initialized_mutex);

        // initialize ddp traj calculation if newly initialized
        if (ddp_init && !ddp_init_last) {
            // update our 3dof model with global krang params then calculate initial trajectory
            updateSimple(threeDOF);
            computeDDPTrajectory(threeDOF);
            pthread_mutex_lock(&ddp_traj_rdy_mutex);
                ddp_traj_rdy = true;
            pthread_mutex_unlock(&ddp_traj_rdy_mutex);
        }

        // Update our track variables
        ddp_init_last = ddp_init;
    }
}

