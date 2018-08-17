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


Vector6d g_state = Vector6d::Zero();
Vector2d g_augstate = Vector2d::Zero();
pthread_mutex_t g_state_mutex;
pthread_mutex_t g_augstate_mutex;
pthread_mutex_t g_robot_mutex;

int main() {

    pthread_mutex_init(&g_state_mutex, NULL);
    pthread_mutex_init(&g_augstate_mutex, NULL);
    pthread_mutex_init(&g_robot_mutex, NULL);

    SkeletonPtr threeDOF = create3DOF_URDF();
    Vector6d g_state = Vector6d::Zero();
    Vector2d g_augstate = Vector2d::Zero();

    readMDPConfig();
    double time_start = get_time();
    computeDDPTrajectory(threeDOF);
    double time_end = get_time();
    cout << "computing DDP took " << time_edd - time_start << " seconds" << endl;

    return 0;
}