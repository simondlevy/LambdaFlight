/****************************************************************************

  custom_physics -- A custom physics model for Crazyflie.


 ******************************************************************************/

#include <plugins/physics.h>

#include "custom2b.h"
#include "custommodel.h"
#include "utils.h"

// constants
const char kRobotName[] = "custom_crazyflie";
const dReal kRotWebotsToAeroBody[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
const dReal kRotAeroToWebotsBody[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// globals
static dBodyID gRobotBody = NULL;

#include <stdio.h>

/*
static FILE * logfp;

static void report(void)
{
    static long count;
    static long prev;
    int curr = time(0);
    if (curr - prev >= 1) {
        fprintf(logfp, "%ld\n", count);
        fflush(logfp);
        prev = curr;
        count = 0;
    }
    count++;
}
*/


//-----------------------------------------------------------------
// Physics plug-in implementation

DLLEXPORT void webots_physics_init() 
{
    //logfp = fopen("/home/levys/Desktop/log.txt", "w");

    // init global variables
    gRobotBody = dWebotsGetBodyFromDEF(kRobotName);
    if (gRobotBody == NULL)
        dWebotsConsolePrintf("!!! custom_physics :: webots_physics_init :: error : could not get body of robot.\r\n");
    else {
        // disable gravity for the custom: buoyancy counteract gravity.
        dBodySetGravityMode(gRobotBody, 0);
    }
}

DLLEXPORT void webots_physics_step() 
{
    // report();

    // we return if we have no robot to actuate.
    if (gRobotBody == NULL) {
        return;
    }

    static float z;

    z = z == 0 ? 1 : z + 0.01;

    dBodySetPosition(gRobotBody, -1, 1, z);
}

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) {
    // We don't want to handle collisions
    return 0;
}

DLLEXPORT void webots_physics_cleanup() {
}
