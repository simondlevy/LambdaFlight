/****************************************************************************

  custom_physics -- A custom physics model for Crazyflie.


 ******************************************************************************/

#include <plugins/physics.h>

#include "custom2b.h"
#include "custommodel.h"
#include "utils.h"

#include "dynamics/fixedpitch/quadxcf.hpp"

// constants
const char ROBOT_NAME[] = "custom_crazyflie";

// globals
static dBodyID _robotBody = NULL;

static FILE * logfp;

//-----------------------------------------------------------------
// Physics plug-in implementation

DLLEXPORT void webots_physics_init() 
{
    logfp = fopen("/home/levys/Desktop/log.txt", "w");

    // init global variables
    _robotBody = dWebotsGetBodyFromDEF(ROBOT_NAME);

    // disable gravity for the custom: buoyancy counteract gravity.
    dBodySetGravityMode(_robotBody, 0);
}

DLLEXPORT void webots_physics_step() 
{
    // report();

    // we return if we have no robot to actuate.
    if (_robotBody == NULL) {
        return;
    }

    // read motor values sent by main program
    int size = 0;
    auto motorvals = (float *)dWebotsReceive(&size);
    fprintf(logfp, "%ld\n", size / sizeof(float));
    fflush(logfp);

    //static float z;
    //z = z == 0 ? 1 : z + 0.001;
    //dBodySetPosition(_robotBody, -1, 1, z);

    dMatrix3 rot = {};
    dBodySetRotation(_robotBody, rot);
}

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) {
    // We don't want to handle collisions
    return 0;
}

DLLEXPORT void webots_physics_cleanup() {
}
