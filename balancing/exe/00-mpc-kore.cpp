/**
 * @file 01-balance.cpp
 * @author Munzir Zafar
 * @date July 26, 2013
 * @brief This code implements the balancing with force-compensations along with basic joystick 
 * control mainly for the demo on July 31st, 2013 using the newly developed kore library.
 */

#include "helpers.h"
#include "kore/display.hpp"
#include "file_ops.hpp"
#include "utils.h"
#include "mpc_ddp.h"
#include "MyWindow.hpp"

using namespace std;
using namespace Krang;

/// The vector of states
vector <LogState*> logStates;

// Debug flags default values
bool debugGlobal = false, logGlobal = true;
bool g_simulation;
bool g_hardwarePos;
Eigen::Matrix<double, 24, 1> g_simInitPos;




/* ********************************************************************************************* */
// The preset arm configurations: forward, thriller, goodJacobian
double presetArmConfs [][7] = {
  {  0.500, -0.600,  0.000, -1.000,  0.000, -1.450,  0.570},
  { -0.500,  0.600,  0.000,  1.000,  0.000,  1.450, -0.480},
  {  1.130, -1.000,  0.000, -1.570, -0.000,  1.000,  -1.104},
  { -1.130,  1.000, -0.000,  1.570,  0.000, -1.000,  -0.958},
  {  1.400, -1.000,  0.000, -0.800,  0.000, -0.500,  -1.000}, 
  { -1.400,  1.000,  0.000,  0.800,  0.000,  0.500,  -1.000},
  {  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
  {  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000},
};

/* ********************************************************************************************* */
/// Controls the arms
void controlArms () {

	// Return if the x[3] is being used for robotiq hands
	if(fabs(x[3]) > 0.2) return;

	// Check if one of the preset configurations are requested by pressing 9 and
	// any of the buttons from 1 to 4 at the same time
	if(((b[4] == 1) && (b[6] == 1)) || ((b[5] == 1) && (b[7] == 1))) {

		// Check if the button is pressed for the arm configuration is pressed, if so send pos commands
		bool noConfs = true;
		for(size_t i = 0; i < 4; i++) {
			if(b[i] == 1) {
				if((b[4] == 1) && (b[6] == 1)) 
					somatic_motor_cmd(&daemon_cx, krang->arms[LEFT], POSITION, presetArmConfs[2*i], 7, NULL);
				if((b[5] == 1) && (b[7] == 1))  {
					somatic_motor_cmd(&daemon_cx, krang->arms[RIGHT], POSITION, presetArmConfs[2*i+1], 7, NULL);
				}
				noConfs = false; 
				return;
			}
		}
		
		// If nothing is pressed, stop the arms
		if(noConfs) {
			double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			somatic_motor_cmd(&daemon_cx, krang->arms[LEFT], VELOCITY, dq, 7, NULL);
			somatic_motor_cmd(&daemon_cx, krang->arms[RIGHT], VELOCITY, dq, 7, NULL);
			return;
		}
	}
	
	// Check the b for each arm and apply velocities accordingly
	// For left: 4 or 6, for right: 5 or 7, lower arm button is smaller (4 or 5)
	somatic_motor_t* arm [] = {krang->arms[LEFT], krang->arms[RIGHT]};
	for(size_t arm_idx = 0; arm_idx < 2; arm_idx++) {

		// Initialize the input
		double dq [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// Change the input based on the lower or higher button input
		bool inputSet = true;
		size_t lowerButton = 4 + arm_idx, higherButton = 6 + arm_idx;
		if(b[lowerButton] && !b[higherButton]) memcpy(&dq[4], x, 3*sizeof(double));
		else if(!b[lowerButton] && b[higherButton]) memcpy(dq, x, 4*sizeof(double));
		else inputSet = false;
		
		// Set the input for this arm
		if(inputSet) somatic_motor_cmd(&daemon_cx, arm[arm_idx], VELOCITY, dq, 7, NULL);
	}
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the waist module
void controlWaist() {

	// Set the mode we want to send to the waist daemon
	Somatic__WaistMode waistMode;
	if(x[5] < -0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_FWD;
	else if(x[5] > 0.9) waistMode = SOMATIC__WAIST_MODE__MOVE_REV;
	else waistMode = SOMATIC__WAIST_MODE__STOP;

	// Send message to the krang-waist daemon
	somatic_waist_cmd_set(waistDaemonCmd, waistMode);
	int r = SOMATIC_PACK_SEND(krang->waistCmdChan, somatic__waist_cmd, waistDaemonCmd);
	if(ACH_OK != r) fprintf(stderr, "Couldn't send message: %s\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)));
}

/* ********************************************************************************************* */
/// Handles the joystick commands for the left/right Schunk grippers
void controlSchunkGrippers () {

	// Button 4 with top/down at the right circular thingy indicates a motion for the left gripper
	double dq [] = {0.0};
	dq[0] = x[3] * 10.0;
	if(b[4]) 
		somatic_motor_cmd(&daemon_cx, krang->grippers[LEFT], SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, dq, 1, NULL);

	// Button 5 with the same circular thingy for the right gripper
	if(b[5]) 
		somatic_motor_cmd(&daemon_cx, krang->grippers[RIGHT], SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, dq, 1, NULL);
}

void controlWheels(double* input) {
	if(start) {
		if (g_simulation) {
			double km = 12.0 * 0.00706; // 12 (oz-in/A) * 0.00706 (Nm/oz-in)
			// Gear Ratio
			// page 2, row "GBPH-0902-NS-015-xxxxx-yyy" of:
			// https://www.anaheimautomation.com/manuals/gearbox/L010455%20-%20GBPH-090x-NS%20Series%20Spec%20Sheet.pdf
			double GR = 15;
			tau_L = input[0] * GR * km;
			tau_R = input[0] * GR * km;

			Eigen::Matrix<double, 2, 1> mForces;
			mForces(0) = tau_L;
			mForces(1) = tau_R;
			const vector<size_t> index{6, 7};
			g_robot->setForces(index, mForces);
		} else {
			if(debug) cout << "Started..." << endl;
			somatic_motor_cmd(&daemon_cx, krang->amc, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, input, 2, NULL);
		}
	}
}

/* ********************************************************************************************* */
/// Handles the wheel commands if we are started
void balanceControl(bool debug, Vector6d& error, double& lastUleft, double& lastUright) {

	// Compute the current
	double u_theta = K.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());
	double u_x = K(2)*error(2) + K(3)*error(3);
	double u_spin =  -K.bottomLeftCorner<2,1>().dot(error.bottomLeftCorner<2,1>());
	u_spin = max(-30.0, min(30.0, u_spin));

	// Compute the input for left and right wheels
	if(joystickControl && ((MODE == GROUND_LO) || (MODE == GROUND_HI))) {u_x = 0.0; u_spin = 0.0;}
	double input [2] = {u_theta + u_x + u_spin, u_theta + u_x - u_spin};
	input[0] = max(-49.0, min(49.0, input[0]));
	input[1] = max(-49.0, min(49.0, input[1]));
	if(debug) printf("u_theta: %lf, u_x: %lf, u_spin: %lf\n", u_theta, u_x, u_spin);
	lastUleft = input[0], lastUright = input[1];

	controlWheels(input);
}

/* ********************************************************************************************* */
/// Update Krang Mode based on configuration, state and state error, updates the K matrices used to calculate u/
void updateKrangMode(Vector6d& error, size_t& mode4iter, Vector6d& state) {
	size_t mode4iterLimit = 100;
	// If in ground Lo mode and waist angle increases beyond 150.0 goto groundHi mode
	if(MODE == GROUND_LO) {
		if((krang->waist->pos[0]-krang->waist->pos[1])/2.0 < 150.0*M_PI/180.0) {
			changeMODE(GROUND_HI);
			K = K_groundHi;
		}
	}
		// If in ground Hi mode and waist angle decreases below 150.0 goto groundLo mode
	else if(MODE == GROUND_HI) {
		if((krang->waist->pos[0]-krang->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
			changeMODE(GROUND_LO);
			K = K_groundLo;
		}
	}

		// If we are in the sit down mode, over write the reference
	else if(MODE == SIT) {
		static const double limit = ((-103.0 / 180.0) * M_PI);
		if(krang->imu < limit) {
			printf("imu (%lf) < limit (%lf): changing to mode 1\n", krang->imu, limit);
			changeMODE(GROUND_LO);
			K = K_groundLo;
		}
		else error(0) = krang->imu - limit;
	}
		// if in standing up mode check if the balancing angle is reached and stayed, if so switch to balLow mode
	else if(MODE == STAND) {
		if(fabs(state(0)) < 0.034) mode4iter++;
		// Change to mode 4 (balance low) if stood up enough
		if(mode4iter > mode4iterLimit) {
			changeMODE(BAL_LO);
			mode4iter = 0;
			K = K_balLow;
		}
	}
		// COM error correction in balLow mode
	else if(MODE == BAL_LO) {
		// error(0) += 0.005;
	}
		// COM error correction in balHigh mode
	else if(MODE == BAL_HI) {
		// error(0) -= 0.005;
	}
}

/* ********************************************************************************************* */
/// Handles the torso commands if we are using joystick
void controlTorso() {
	// Control the torso
	double dq [] = {x[4] / 7.0};
	somatic_motor_cmd(&daemon_cx, krang->torso, VELOCITY, dq, 1, NULL);
}

/* ********************************************************************************************* */
/// Handles the wheel commands if we are started
bool controlStandSit(Vector6d& error, Vector6d& state) {
	static int lastb9 = b[9];
	// ==========================================================================
	// Quit if button 9 on the joystick is pressed, stand/sit if button 10 is pressed
	// Quit
	if(b[8] == 1) return true;

	// pressing button 10 should initiate stand or sit if button 6 is not pressed
	// and should toggle MPC mode if button 6 is pressed
	if(b[5] == 0) {
		if(b[9] == 1 && lastb9 == 0) {
			// If in ground mode and state error is not high stand up
			if(MODE == GROUND_LO) {
				if(state(0) < 0.0 && error(0) > -10.0*M_PI/180.0)	{
					printf("\n\n\nMode 2\n\n\n");
					K = K_stand;
					changeMODE(STAND);
				}	else {
					printf("\n\n\nCan't stand up, balancing error is too high!\n\n\n");
				}
			}

				// If in balLow mode and waist is not too high, sit down
			else if(MODE == STAND || MODE == BAL_LO || MODE == MPC_M) {
				if((krang->waist->pos[0] - krang->waist->pos[1])/2.0 > 150.0*M_PI/180.0) {
					printf("\n\n\nMode 3\n\n\n");
					K = K_sit;
					changeMODE(SIT);
				} else {
					printf("\n\n\nCan't sit down, Waist is too high!\n\n\n");
				}
			}
		}
	}
	else {
		if(b[9] == 1 && lastb9 == 0) {
			bool is_initialized = false;
		    pthread_mutex_lock(&ddp_initialized_mutex);
		        is_initialized = ddp_initialized;
		    pthread_mutex_unlock(&ddp_initialized_mutex);

			if(!is_initialized) initializeMPCDDP();
			else exitMPC();
		}
	}
	lastb9 = b[9];
	return false;
}

/* ******************************************************************************************** */
/// The continuous control loop which has 4 state variables, {x, x., psi, psi.}, where
/// x is for the position along the heading direction and psi is the heading angle. We create
/// reference x and psi values from the joystick and follow them with pd control.
void run () {

	// Send a message; set the event code and the priority
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
			SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// Initially the reference position and velocities are zero (don't move!) (and error!)
	// Initializing here helps to print logs of the previous state
	Vector6d refState = Vector6d::Zero(), state = Vector6d::Zero(), error = Vector6d::Zero();

	// Augment two state to represent x0 and y0.
	g_augstate = Vector2d::Zero(); g_xInit = 0.0; g_psiInit = 0.0;

	// Read the FT sensor wrenches, shift them on the wheel axis and display
	size_t c_ = 0;
	struct timespec t_now, t_prev = aa_tm_now();
	double time = 0.0;
	double t_now_2_sec;
	Vector6d externalWrench;
	Vector3d com;
	bool use_mpc_traj = false;

	// Initialize the running history
	const size_t historySize = 60;

	// Continue processing data until stop received
	double js_forw = 0.0, js_spin = 0.0, averagedTorque = 0.0, lastUleft = 0.0, lastUright = 0.0;
	size_t mode4iter = 0;
	KRANG_MODE lastMode = MODE; bool lastStart = start;

	SkeletonPtr threeDOF = create3DOF_URDF();

	bool firstIteration = true;

	while(!somatic_sig_received) {

		if(g_simulation) {
			if(!firstIteration) {
				pthread_mutex_lock(&simSync_mutex1);
				pthread_mutex_unlock(&simSync_mutex2);
			}
			firstIteration = false;
		}


		bool debug = (c_++ % 20 == 0);
		debugGlobal = debug;
		if(debug) cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv\n" << endl;

		// Cancel any position built up in previous mode
		if(lastMode != MODE) {
			refState(2) = state(2), refState(4) = state(4);
			lastMode = MODE;
		}
		if(lastStart != start) {
			refState(2) = state(2), refState(4) = state(4);
			lastStart = start;
		}

		// =======================================================================
		// Get inputs: time, joint states, joystick and external forces

		// Get the current time and compute the time difference and update the prev. time
		t_now = aa_tm_now();						
		double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));
		t_now_2_sec = (double)aa_tm_timespec2sec(t_now);
		t_prev = t_now;
		time += dt;

		if (g_simulation) {
		    // does the dt matter?
			getSimState(state, &com);
		} else {
			// Get the current state and ask the user if they want to start
			getState(state, dt, &com);
		}
		updateAugState(state, dt);     // Update Augmented State

		if(debug) {
			cout << "\nstate: " << state.transpose() << endl;
			cout << "com: " << com.transpose() << endl;
			cout << "WAIST ANGLE: " << krang->waist->pos[0] << endl;
		}

		// Print the information about the last iteration (after reading effects of it from sensors)
		// NOTE: Constructor order is NOT the print order
		if(logGlobal) {
			logStates.push_back(new LogState(time, com, averagedTorque, externalWrench(4), krang->amc->cur[0],
				krang->amc->cur[1], state, refState, lastUleft, lastUright));
		}

		// Get the joystick input for the js_forw and js_spin axes (to set the gains)
		bool gotInput = false;
		while(!gotInput) gotInput = getJoystickInput(js_forw, js_spin);
		if(debug) cout << "js_forw: " << js_forw << ", js_spin: " << js_spin << endl;

		// =======================================================================
		// Compute ref state using joystick

		// Update the reference values for the position and spin
		updateReference(js_forw, js_spin, dt, refState);
		if(debug) cout << "refState: " << refState.transpose() << endl;

		// =======================================================================
		// Apply control: compute error, threshold and send current

		// Compute the error term between reference and current, and weight with gains (spin separate)
		if(debug) cout << "K: " << K.transpose() << endl;

		error = state - refState;
		if(debug) cout << "error: " << error.transpose() << ", imu: " << krang->imu / M_PI * 180.0 << endl;

		// =======================================================================
		// Control the arms, waist torso and robotiq grippers based on the joystick input

		// If we are in joystick mode and not using MPC
		if (MODE != MPC_M) {
			updateKrangMode(error, mode4iter, state);

			balanceControl(debug, error, lastUleft, lastUright);

			if(joystickControl) {
				if(debug) cout << "Joystick for Arms and Waist..." << endl;
				controlArms();
				controlWaist();
				controlTorso();
				if(controlStandSit(error, state)) return;
			}
		}

		// Else are in mpc mode
		else {

			// find the appropriate index from mpc trajetories
			double time_now = get_time();
			double mpc_init_time = get_mpc_init_time();
			int MPCStepIdx = floor((time_now - mpc_init_time) / g_mpcConfig.MPCdt);
			double tf = g_mpcConfig.finalTime;

			if (time_now < (mpc_init_time + tf)) {
				Control u;
				DDPDynamics *mpc_dynamics;
				State cur_state, xdot;
				Eigen::Vector3d ddq, dq;
				c_forces dy_forces;
				double input[2];
				double ddth, tau_0, ddx, ddpsi, tau_1, tau_L, tau_R;

				// =============== Read mpc control step
				bool mpc_reading_done = false;
				while(!mpc_reading_done) {
					if (pthread_mutex_trylock(&g_mpc_trajectory_main_mutex) == 0) {
						u = g_mpc_trajectory_main.col(MPCStepIdx);  // Using counter to get the correct reference
						pthread_mutex_unlock(&g_mpc_trajectory_main_mutex);
						mpc_reading_done = true;
					} 
					else if (pthread_mutex_trylock(&g_mpc_trajectory_backup_mutex) == 0) {
						u = g_mpc_trajectory_main.col(MPCStepIdx);  // Using counter to get the correct reference
						pthread_mutex_unlock(&g_mpc_trajectory_backup_mutex);
						mpc_reading_done = true;
					}
				}

				
				// =============== Spin Torque tau_0
				tau_0 = u(1);


				// =============== Forward Torque tau_1
				
				// ddthref from High-level Control
				ddth = u(0);
				
				// Get dynamics object 
				mpc_dynamics = getDynamics(threeDOF);
				
				// current state
				pthread_mutex_lock(&g_state_mutex);
				pthread_mutex_lock(&g_augstate_mutex);
				cur_state << 0.25*g_state(2)-g_xInit, g_state(4)-g_psiInit, g_state(0), 0.25*g_state(3), g_state(5), g_state(1), g_augstate(0), g_augstate(1);
				pthread_mutex_unlock(&g_state_mutex);
				pthread_mutex_unlock(&g_augstate_mutex);

				xdot = mpc_dynamics->f(cur_state, u);
				ddx = xdot(3);
				ddpsi = xdot(4);
				ddq << ddx, ddpsi, ddth;

				// dq
				dq = cur_state.segment(3, 3);

				// A, C, Q and Gamma_fric
				dy_forces = mpc_dynamics->dynamic_forces(cur_state, u);

				// tau_1
				tau_1 = dy_forces.A.block<1, 3>(2, 0) * ddq;
				tau_1 += dy_forces.C.block<1, 3>(2, 0) * dq;
				tau_1 += dy_forces.Q(2);
				tau_1 -= dy_forces.Gamma_fric(2);

				// =============== Wheel Torques
				tau_L = -0.5 * (tau_1 + tau_0);
				tau_R = -0.5 * (tau_1 - tau_0);
				
				// =============== Torque to current conversion

				// Motor Constant
				// page 2, row "BLY343D-24V-2000" of:
				// https://www.anaheimautomation.com/manuals/brushless/L010350%20-%20BLY34%20Series%20Product%20Sheet.pdf
				double km = 12.0 * 0.00706; // 12 (oz-in/A) * 0.00706 (Nm/oz-in)

				// Gear Ratio
				// page 2, row "GBPH-0902-NS-015-xxxxx-yyy" of:
				// https://www.anaheimautomation.com/manuals/gearbox/L010455%20-%20GBPH-090x-NS%20Series%20Spec%20Sheet.pdf
				double GR = 15;

				input[0] = min(49.5, max(-49.5, tau_L/GR/km));
				input[1] = min(49.5, max(-49.5, tau_R/GR/km));
				lastUleft = input[0], lastUright = input[1];

				// =============== Set the Motor Currents
				controlWheels(input);
			}
			else {
				exitMPC();
			}
		}

	// Print the mode
		if(debug) printf("Mode : %d\tdt: %lf\n", MODE, dt);

		if(g_simulation) {
			pthread_mutex_unlock(&simSync_mutex1);
			pthread_mutex_lock(&simSync_mutex2);
		}
	}

	// Send the stoppig event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE,
					 SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
/// Initialize the motor and daemons
void init(int argc, char* argv[]) {

	// Initialize the daemon
	somatic_d_opts_t dopt;
	memset(&dopt, 0, sizeof(dopt)); 
	dopt.ident = "01-balance";
	somatic_d_init(&daemon_cx, &dopt);

	// Initialize the motors and sensors on the hardware and update the kinematics in dart
	int hwMode = Krang::Hardware::MODE_AMC | Krang::Hardware::MODE_LARM | 
		Krang::Hardware::MODE_RARM | Krang::Hardware::MODE_TORSO | Krang::Hardware::MODE_WAIST;
	krang = new Krang::Hardware((Krang::Hardware::Mode) hwMode, &daemon_cx, g_robot);

	// Initialize the joystick channel
	int r = ach_open(&js_chan, "joystick-data", NULL);
	aa_hard_assert(r == ACH_OK, "Ach failure '%s' on opening Joystick channel (%s, line %d)\n", 
		ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Read DDP config file
	readMDPConfig();


	// initialize ddp related mutex
	pthread_mutex_init(&ddp_initialized_mutex, NULL);

	pthread_mutex_init(&g_mpc_init_time_mutex, NULL);
	pthread_mutex_init(&g_mpc_trajectory_main_mutex, NULL);
	pthread_mutex_init(&g_mpc_trajectory_backup_mutex, NULL);

	pthread_mutex_init(&g_state_mutex, NULL);
	pthread_mutex_init(&g_augstate_mutex, NULL);
	pthread_mutex_init(&g_robot_mutex, NULL);

	pthread_mutex_init(&MODE_mutex, NULL);

	// Create a thread to wait for user input to begin balancing
	pthread_t kbhitThread;
	pthread_create(&kbhitThread, NULL, &kbhit, NULL);

	pthread_t mpcddpThread;
	pthread_create(&mpcddpThread, NULL, &mpcddp, NULL);

	// *********************************** See if simulation mode is specified
	Configuration *  cfg = Configuration::create();
	const char *     scope = "";
	const char *     configFile = "/home/munzir/project/krang/28-balance-kore/balancing/src/controlParams.cfg";
	const char * str;
	std::istringstream stream;
	double newDouble;

	try {
		cfg->parse(configFile);

		// str = cfg->lookupString(scope, "goalState"); 
		// stream.str(str); for(int i=0; i<8; i++) stream >> mGoalState(i); stream.clear();

		g_simulation = cfg->lookupBoolean(scope, "simulation");
		cout << "Simulation: " << (g_simulation?"true":"false") << endl;
		g_hardwarePos = cfg->lookupBoolean(scope, "simulateInitPositionsFromHardware");
		cout << "Simulation Init Pos From Hardware: " << (g_hardwarePos?"true":"false") << endl;
		str = cfg->lookupString(scope, "simulationInitPositions");
		stream.str(str); for(int i=0; i<24; i++) stream >> g_simInitPos; stream.clear();

	} catch(const ConfigurationException & ex) {
		cerr << ex.c_str() << endl;
		cfg->destroy();
	}

	pthread_t simThread;
	if(g_simulation) {
		struct simArguments simArgs;
		simArgs.argc = argc;
		simArgs.argv = &argv[0];
		simArgs.world = world;
		simArgs.robot = g_robot;
		pthread_mutex_init(&simSync_mutex1, NULL);
		pthread_mutex_init(&simSync_mutex2, NULL);
		pthread_mutex_lock(&simSync_mutex1);
		if (g_hardwarePos) {
			krang->updateSensors(0.001);
		}
		if (g_simInitPos) {
			setInitPos();
		}
		pthread_create(&simThread, NULL, &simfunc, &simArgs);
	}
}

void setInitPos() {
	double headingInit, qBaseInit, qLWheelInit, qRWheelInit, qWaistInit, qTorsoInit, qKinectInit, th;
	Eigen::Vector3d xyzInit;
	Eigen::Matrix<double, 7, 1> qLeftArmInit;
	Eigen::Matrix<double, 7, 1> qRightArmInit;
	Eigen::Transform<double, 3, Eigen::Affine> baseTf;
	Eigen::AngleAxisd aa;
	Eigen::Matrix<double, 25, 1> q;

	headingInit = g_simInitPos(0);
	qBaseInit = g_simInitPos(1);
	xyzInit << g_simInitPos.segment(2,3);
	qLWheelInit = g_simInitPos(5);
	qRWheelInit = g_simInitPos(6);
	qWaistInit = g_simInitPos(7);
	qTorsoInit = g_simInitPos(8);
	qKinectInit = g_simInitPos(9);
	qLeftArmInit << g_simInitPos.segment(10, 7);
	qRightArmInit << g_simInitPos.segment(17, 7);
	// Calculating the axis angle representation of orientation from headingInit and qBaseInit:
	// RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
	baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
	baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
	aa = Eigen::AngleAxisd(baseTf.rotation());

	// Set the positions and get the resulting COM angle
	q << aa.angle()*aa.axis(), xyzInit, qLWheelInit, qRWheelInit, qWaistInit, qTorsoInit, qKinectInit, qLeftArmInit, qRightArmInit;
	g_robot->setPositions(q);
}

/* ******************************************************************************************** */
/// Send zero velocity to the motors and kill daemon. Also clean up daemon structs.
void destroy() {

	cout << "destroying" << endl;

	// ===========================
	// Stop motors, close motor/sensor channels and destroy motor objects
	
	// To prevent arms from halting if joystick control is not on, change mode of krang
	if(!joystickControl) {
		somatic_motor_destroy(&daemon_cx, krang->arms[LEFT]);
		somatic_motor_destroy(&daemon_cx, krang->arms[RIGHT]);
	  krang->arms[LEFT] = NULL;
	  krang->arms[RIGHT] = NULL;
	}
	delete krang;
		
	// Destroy the daemon resources
	somatic_d_destroy(&daemon_cx);

	// Print the data
	printf("log states size: %lu\n", logStates.size());
	for(size_t i = 0; i < logStates.size(); i++) {
		logStates[i]->print();
		delete logStates[i];
	}
}

/* ******************************************************************************************** */
// // Change robot's beta values (parameters)
void setParameters(Eigen::MatrixXd betaParams, int bodyParams) {
	Eigen::Vector3d bodyMCOM;
	double mi;
	int numBodies = betaParams.cols()/bodyParams;
	for (int i = 0; i < numBodies; i++) {
		cout << "0.1 - " << i << endl;
		mi = betaParams(0, i * bodyParams);
		bodyMCOM(0) = betaParams(0, i * bodyParams + 1);
		bodyMCOM(1) = betaParams(0, i * bodyParams + 2);
		bodyMCOM(2) = betaParams(0, i * bodyParams + 3);

		g_robot->getBodyNode(i)->setMass(mi);
		g_robot->getBodyNode(i)->setLocalCOM(bodyMCOM/mi);
	}
}

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	Eigen::MatrixXd beta;
	// Load the world and the robot
	dart::utils::DartLoader dl;
	g_robot = dl.parseSkeleton("/home/munzir/project/krang/09-URDF/Krang/KrangNoKinect.urdf");
	g_robot->setName("krang");
	assert((g_robot != NULL) && "Could not find the robot urdf");
	string inputBetaFilename = "../convergedBetaVector104PosesHardwareTrained.txt";

	try {
		cout << "Reading converged beta ...\n";
		beta = readInputFileAsMatrix(inputBetaFilename);
		cout << "|-> Done\n";
	} catch (exception& e) {
		cout << e.what() << endl;
		return EXIT_FAILURE;
	}
	setParameters(beta, 4);
	world = std::make_shared<World>();
	world->addSkeleton(g_robot);

	//Read Gains from file
	readGains();

	// Debug options from command line
	debugGlobal = 1; logGlobal = 0;
	if(argc == 8) {
		if(argv[7][0]=='l') { debugGlobal = 0; logGlobal = 1;} 
		else if(argv[7][0] == 'd') {debugGlobal = 1; logGlobal = 0; } 
	} 

	getchar();

	// Initialize, run, destroy
	init(argc, &argv[0]);
	run();
	destroy();
	return 0;
}
