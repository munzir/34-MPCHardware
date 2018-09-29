/**
 * @file helpers.cpp
 * @author Can Erdogan, Peng Hou
 * @date July 08, 2013
 * @brief This file contains helper functions such as imu data retrieval and -+++++++++ing...
 */

#include "helpers.h"
#include "mpc_ddp.h"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace Eigen;
using namespace std;
using namespace Krang;

/* ******************************************************************************************** */
// Initialize the gains for controller and joystick
KRANG_MODE MODE = GROUND_LO;

Vector6d K_groundLo;
Vector6d K_groundHi;
Vector2d J_ground (1.0, 1.0);
Vector6d K_stand;
Vector2d J_stand;
Vector6d K_sit;
Vector2d J_sit;
Vector6d K_balLow;
Vector2d J_balLow;
Vector6d K_balHigh;
Vector2d J_balHigh;
Vector6d K = K_groundLo;

/* ******************************************************************************************** */
// Constants for the robot kinematics
const double wheelRadius = 10.5; 							///< Radius of krang wheels in inches
const double distanceBetweenWheels = 27.375; 	///< Distance Between krang's wheels in inches

/* ******************************************************************************************** */
// Initialize global variables

somatic_d_t daemon_cx;				///< The context of the current daemon
Somatic__WaistCmd *waistDaemonCmd = somatic_waist_cmd_alloc();
bool start = false;						///< Giving time to the user to get the robot in balancing angle
bool joystickControl = false;

Krang::Hardware* krang;				///< Interface for the motor and sensors on the hardware
WorldPtr world;			///< the world representation in dart
SkeletonPtr g_robot;			///< the robot representation in dart

ach_channel_t js_chan;				///< Read joystick data on this channel

double jsFwdAmp;				///< The gains for joystick forward/reverse input
double jsSpinAmp;				///< The gains for joystick left/right spin input

char b [10];						///< Stores the joystick button inputs
double x [6];

bool ddp_initialized = false;
pthread_mutex_t ddp_initialized_mutex;

// DDP related global variables shared between threads
Vector6d g_state = Vector6d::Zero();
Vector2d g_augstate = Vector2d::Zero();
double g_xInit, g_psiInit;
pthread_mutex_t g_state_mutex;
pthread_mutex_t g_augstate_mutex;
pthread_mutex_t g_robot_mutex;
pthread_mutex_t MODE_mutex;


/* ******************************************************************************************** */
// fixer function which removes values from matrix below threshold
Eigen::MatrixXd fix (const Eigen::MatrixXd& mat) {
	double threshold = 1e-5;
	Eigen::MatrixXd mat2 (mat);
	for(size_t i = 0; i < mat2.rows(); i++)
		for(size_t j = 0; j < mat2.cols(); j++)
			if(fabs(mat2(i,j)) < threshold) mat2(i,j) = 0.0;
	return mat2;
}


/* ******************************************************************************************** */
/// Updates global state with the given state
void changeMODE(KRANG_MODE m) {
    pthread_mutex_lock(&MODE_mutex);
    MODE = m;
	pthread_mutex_unlock(&MODE_mutex);
}


/* ******************************************************************************************** */
/// Updates global state with the given state
void updateGlobalState(Vector6d &state) {
    pthread_mutex_lock(&g_state_mutex);
	for (int i = 0; i<6; i++) {
		g_state(i) = state(i);
	}
	pthread_mutex_unlock(&g_state_mutex);
}

/* ******************************************************************************************** */
/// Get the joint values from the encoders and the imu and compute the center of mass as well
void getState(Vector6d& state, double dt, Vector3d* com_) {

	pthread_mutex_lock(&g_robot_mutex);
	// Read motor encoders, imu and ft and update dart skeleton
	krang->updateSensors(dt);

	// Calculate the COM Using Skeleton
  	Vector3d com = g_robot->getCOM() - g_robot->getPositions().segment(3,3);
	pthread_mutex_unlock(&g_robot_mutex);

  	if(com_ != NULL) *com_ = com;

	// Update the state (note for amc we are reversing the effect of the motion of the upper body)
	// State are theta, dtheta, x, dx, psi, dpsi
	state(0) = atan2(com(0), com(2)); // - 0.3 * M_PI / 180.0;;
	state(1) = krang->imuSpeed;
	state(2) = (krang->amc->pos[0] + krang->amc->pos[1])/2.0 + krang->imu;
	state(3) = (krang->amc->vel[0] + krang->amc->vel[1])/2.0 + krang->imuSpeed;
	state(4) = (krang->amc->pos[1] - krang->amc->pos[0]) / 2.0;
	state(5) = (krang->amc->vel[1] - krang->amc->vel[0]) / 2.0;

	// Making adjustment in com to make it consistent with the hack above for state(0)
	com(0) = com(2) * tan(state(0));
}

/* ******************************************************************************************** */
/// Update Augment State where dt is last iter time
void updateAugState(Vector6d& state, double dt){

	double R = 0.25;  /// Radius of wheel 25cm;
	pthread_mutex_lock(&g_augstate_mutex);
		g_augstate(0) = g_augstate(0) + dt*(R*state(3)*cos(state(4)-g_psiInit));   //x0 = x0 + dt*dx0; and dx0 = dx*cos(psi);
		g_augstate(1) = g_augstate(1) + dt*(R*state(3)*sin(state(4)-g_psiInit));   //y0 = y0 + dt*dy0; and dy0 = dx*sin(psi);
    pthread_mutex_unlock(&g_augstate_mutex);
}

/* ******************************************************************************************** */
/// Update reference left and right wheel pos/vel from joystick data where dt is last iter. time
void updateReference (double js_forw, double js_spin, double dt, Vector6d& refState) {

	// First, set the balancing angle and velocity to zeroes
	refState(0) = refState(1) = 0.0;

	// Set the distance and heading velocities using the joystick input
	refState(3) = js_forw;
	refState(5) = js_spin;

	// Integrate the reference positions with the current reference velocities
	refState(2) += dt * refState(3);
	refState(4) += dt * refState(5);
}

/* ******************************************************************************************** */
/// Returns the values of axes 1 (left up/down) and 2 (right left/right) in the joystick
bool getJoystickInput(double& js_forw, double& js_spin) {

	// Get the message and check output is OK.
	int r = 0;
	Somatic__Joystick *js_msg =
			SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, NULL, 4096, &js_chan );
	if(!(ACH_OK == r || ACH_MISSED_FRAME == r) || (js_msg == NULL)) return false;

	// Get the values
	for(size_t i = 0; i < 10; i++)
		b[i] = js_msg->buttons->data[i] ? 1 : 0;
	memcpy(x, js_msg->axes->data, sizeof(x));

	// Free the joystick message
	somatic__joystick__free_unpacked(js_msg, NULL);

	// Change the gains with the given joystick input
	double deltaTH = 0.2, deltaX = 0.02, deltaSpin = 0.02;
	if(!joystickControl) {
		for(size_t i = 0; i < 4; i++) {
			if(((b[5] == 0) && (b[7] == 0)) && (b[i] == 1)) K(i % 2) += ((i < 2) ? deltaTH : -deltaTH);
			else if((b[5] == 1) && (b[i] == 1)) K((i % 2) + 2) += ((i < 2) ? deltaX : -deltaX);
			else if((b[7] == 1) && (b[i] == 1)) K((i % 2) + 4) += ((i < 2) ? deltaSpin : -deltaSpin);
		}
	}

	// Update joystick and force-compensation controls
	static int lastb0 = b[0], lastb1 = b[1], lastb2 = b[2];

	if((b[4] == 1) && (b[6] == 0) && (b[0] == 1) && (lastb0 == 0)) {
		joystickControl = !joystickControl;
		if(joystickControl == true) {
			somatic_motor_reset(&daemon_cx, krang->arms[LEFT]);
			somatic_motor_reset(&daemon_cx, krang->arms[RIGHT]);
		}
	}

	if((b[4] == 1) && (b[6] == 0) && (b[2] == 1) && (lastb2 == 0)) {
		if(MODE == BAL_LO) {
			printf("Mode 5\n");
			K = K_balHigh;
			changeMODE(BAL_HI);
		}
		else if (MODE == BAL_HI) {
			printf("Mode 4\n");
			K = K_balLow;
			changeMODE(BAL_LO);
		}
	}

	lastb0 = b[0], lastb1 = b[1], lastb2 = b[2];

	// Ignore the joystick statements for the arm control
	if((b[4] == 1) || (b[5] == 1) || (b[6] == 1) || (b[7] == 1)) {
		js_forw = js_spin = 0.0;
		return true;
	}

	// Set the values for the axis
	double* x = &(js_msg->axes->data[0]);
	if(MODE == GROUND_LO || MODE == GROUND_HI) {
		js_forw = -J_ground(0) * x[1], js_spin = J_ground(1) * x[2];
	}
	else if(MODE == BAL_LO) {
		js_forw = -J_balLow(0) * x[1], js_spin = J_balLow(1) * x[2];
	}
	else if(MODE == BAL_HI) {
		js_forw = -J_balHigh(0) * x[1], js_spin = J_balHigh(1) * x[2];
	}
	else {
		js_forw = -x[1] * jsFwdAmp;
		js_spin = x[2] * jsSpinAmp;;
	}

	return true;
}

/* ******************************************************************************************** */
/// Read file for gains
void readGains () {

	Vector6d* kgains [] = {&K_groundLo, &K_stand, &K_sit, &K_balLow, &K_balHigh, &K_groundHi};
	Vector2d* jgains [] = {&J_ground, &J_stand, &J_sit, &J_balLow, &J_balHigh, &J_ground};
	ifstream file ("../gains.txt");
	assert(file.is_open());
	char line [1024];
	for(size_t k_idx = 0; k_idx < 6; k_idx++) {
		*kgains[k_idx] = Vector6d::Zero();
		*jgains[k_idx] = Vector2d::Zero();
		file.getline(line, 1024);
		std::stringstream stream(line, std::stringstream::in);
		size_t i = 0;
		double newDouble;
		while ((i < 6) && (stream >> newDouble)) (*kgains[k_idx])(i++) = newDouble;
		while (stream >> newDouble) (*jgains[k_idx])(i++ - 6) = newDouble;
	}
	file.close();

	pv(K_groundLo);
	pv(K_groundHi);
	pv(J_ground);
	pv(K_stand);
	pv(J_stand);
	pv(K_sit);
	pv(J_sit);
	pv(K_balLow);
	pv(J_balLow);
	pv(K_balHigh);
	pv(J_balHigh);

	K = K_groundLo;
}

/* ********************************************************************************************* */
/// Sets a global variable ('start') true if the user presses 's'
void *kbhit(void *) {
	char input;
	while(true){
		input=cin.get();
		if(input=='s') start = true;
		else if(input=='.') readGains();
		else if(input=='j') {
			joystickControl = !joystickControl;
			if(joystickControl == true) {
				somatic_motor_reset(&daemon_cx, krang->arms[LEFT]);
				somatic_motor_reset(&daemon_cx, krang->arms[RIGHT]);
			}
		}
		else if(input=='1') {
			printf("Mode 1\n");
			K = K_groundLo;
			changeMODE(GROUND_LO);
		}
		else if(input=='2') {
			printf("Mode 2\n");
			K = K_stand;
			changeMODE(STAND);
		}
		else if(input=='3') {
			printf("Mode 3\n");
			K = K_sit;
			changeMODE(SIT);
		}
		else if(input=='4') {
			printf("Mode 4\n");
			K = K_balLow;
			changeMODE(BAL_LO);
		}
		else if(input=='5') {
			printf("Mode 5\n");
			K = K_balHigh;
			changeMODE(BAL_HI);
		}
		else if(input=='6') {
			printf("Mode 6\n");
			K = K_groundHi;
			changeMODE(GROUND_HI);
		}
		else if (input =='m') {
		    initializeMPCDDP();
		}
		else if (input == 'n') {
			if (MODE !=  MPC_M) {
				printf("Krang is not in MPC mode!\n");
			} else {
				exitMPC();
			}
		}
	}
}

/* ********************************************************************************************* */
/// Computes the imu value from the imu readings
//void getImu (ach_channel_t* imuChan, double& _imu, double& _imuSpeed, double dt,
//		filter_kalman_t* kf) {
//
//	// ======================================================================
//	// Get the readings
//
//	// Get the value
//	int r;
//	struct timespec currTime;
//	clock_gettime(CLOCK_MONOTONIC, &currTime);
//	struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);
//	Somatic__Vector *imu_msg = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector,
//														NULL, IMU_CHANNEL_SIZE, imuChan, &abstime );
//	assert((imu_msg != NULL) && "Imu message is faulty!");
//
//	// Get the imu position and velocity value from the readings (note imu mounted at 45 deg).
//	static const double mountAngle = -.7853981634;
//	double newX = imu_msg->data[0] * cos(mountAngle) - imu_msg->data[1] * sin(mountAngle);
//	_imu = atan2(newX, imu_msg->data[2]);
//	_imuSpeed = imu_msg->data[3] * sin(mountAngle) + imu_msg->data[4] * cos(mountAngle);
//
//	// Free the unpacked message
//	somatic__vector__free_unpacked( imu_msg, NULL);
//
//	// ======================================================================
//	// Filter the readings
//
//	// Skip if a filter is not provided
//	if(kf == NULL) return;
//
//	// Setup the data
//	kf->z[0] = _imu, kf->z[1] = _imuSpeed;
//
//	// Setup the time-dependent process matrix
//	kf->A[0] = kf->A[3] = 1.0;
//	kf->A[2] = dt;
//
//	// Setup the process noise matrix
//	static const double k1 = 2.0;
//	static const double k1b = 5.0;
//	kf->R[0] = (dt*dt*dt*dt) * k1 * (1.0 / 4.0);
//	kf->R[1] = (dt*dt*dt) * k1 * (1.0 / 2.0);
//	kf->R[2] = (dt*dt*dt) * k1 * (1.0 / 2.0);
//	kf->R[3] = (dt*dt) * k1b;
//
//	// First make a prediction of what the reading should have been, then correct it
//	filter_kalman_predict(kf);
//	filter_kalman_correct(kf);
//
//	// Set the values
//	_imu = kf->x[0], _imuSpeed = kf->x[1];
//}
