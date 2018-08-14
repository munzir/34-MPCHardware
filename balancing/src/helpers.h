/**
 * @file helpers.h
 * @author Munzir
 * @date July 8th, 2013
 * @brief This file comtains some helper functions used for balancing	
 */

#pragma once

#include <Eigen/Dense>

#include <dirent.h>
#include <iostream>

#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>

#include <filter.h>
#include <imud.h>
#include <pciod.h>

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <initModules.h>

#include <kore.hpp>

using namespace Eigen;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::common;
using namespace dart::math;

extern bool debugGlobal;

/* ******************************************************************************************** */
typedef Matrix<double, 6, 1> Vector6d;			///< A typedef for convenience to contain f/t values
typedef Matrix<double, 7, 1> Vector7d;			///< A typedef for convenience to contain joint values
typedef Matrix<double, 6, 6> Matrix6d;			///< A typedef for convenience to contain wrenches

/* ******************************************************************************************** */
// Globals for imu, motors and joystick

extern somatic_d_t daemon_cx;				///< The context of the current daemon

extern Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
extern WorldPtr world;			///< the world representation in dart
extern SkeletonPtr robot;			///< the robot representation in dart

extern Somatic__WaistCmd *waistDaemonCmd; ///< Cmds for waist daemon
extern ach_channel_t js_chan;				///< Read joystick data on this channel

extern bool start;						///< Giving time to the user to get the robot in balancing angle
extern bool joystickControl;

extern double jsFwdAmp;				///< The gains for joystick forward/reverse input
extern double jsSpinAmp;				///< The gains for joystick left/right spin input

extern char b [10];						///< Stores the joystick button inputs
extern double x [6];						///< Stores the joystick axes inputs

/* ******************************************************************************************** */
// All the freaking gains

extern size_t MODE;
extern Vector6d K_groundLo;
extern Vector6d K_groundHi;
extern Vector2d J_ground;
extern Vector6d K_stand;
extern Vector2d J_stand;
extern Vector6d K_sit;
extern Vector2d J_sit;
extern Vector6d K_balLow;
extern Vector2d J_balLow;
extern Vector6d K_balHigh;
extern Vector2d J_balHigh;
extern Vector6d K;

/* ******************************************************************************************** */
// Globals for MPC-DDP functions
extern bool ddp_initialized;
extern pthread_mutex_t ddp_initialized_mutex;
extern bool ddp_traj_rdy;
extern pthread_mutex_t ddp_traj_rdy_mutex;

/* ******************************************************************************************** */

/// Makes the small 1e-17 values in a matrix zero for printing
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat);

/* ******************************************************************************************** */
// Constants for end-effector wrench estimation
//static const double eeMass = 2.3 + 0.169 + 0.000;			///< The mass of the robotiq end-effector
static const double eeMass = 1.6 + 0.169 + 0.000;			///< The mass of the Schunk end-effector
static const Vector3d s2com (0.0, -0.008, 0.091); // 0.065 robotiq itself, 0.026 length of ext + 2nd

/* ******************************************************************************************** */
// Helper functions

/// Sets a global variable ('start') true if the user presses 's'
void *kbhit(void *);

/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick 
bool getJoystickInput(double& js_forw, double& js_spin);

/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState);

/// Get the joint values from the encoders and the imu and compute the center of mass as well 
void getState(Vector6d& state, double dt, Vector3d* com = NULL);

/// Updates the dart robot representation
void updateDart (double imu);

/// Reads imu values from the ach channels and computes the imu values
void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt, 
             filter_kalman_t* kf);

void readGains();

/* ******************************************************************************************** */
// Useful macros

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION

// prints vector a
#define pv(a) std::cout << #a << ": " << fix((a).transpose()) << "\n" << std::endl

// prints a as a column
#define pc(a) std::cout << #a << ": " << (a) << "\n" << std::endl

// prints a as a matrix
#define pm(a) std::cout << #a << ":\n " << fix((a).matrix()) << "\n" << std::endl

#define pmr(a) std::cout << #a << ":\n " << fix((a)) << "\n" << std::endl

// Prints llwa pos positions
#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
              llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);

// Print llwa velocities
#define darm (cout << "dq: "<<llwa.vel[0] << ", " <<llwa.vel[1] << ", " << llwa.vel[2] << ", " << \
              llwa.vel[3] << ", " << llwa.vel[4] << ", " << llwa.vel[5] << ", " << llwa.vel[6] << endl);

// Takes first 7 values of x and returns a 7d vector
#define eig7(x) (Vector7d() << (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5], (x)[6]).finished()

/* ******************************************************************************************** */
