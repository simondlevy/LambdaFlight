/****************************************************************************

  custom_physics -- A custom physics model for Crazyflie.


 ******************************************************************************/

#include <plugins/physics.h>

#include "custom2b.h"
#include "custommodel.h"
#include "utils.h"

#include "dynamics/fixedpitch/quadxcf.hpp"

#define LOGFILE "/home/levys@ad.wlu.edu/Desktop/log.txt"

// constants
const char ROBOT_NAME[] = "custom_crazyflie";

// globals
static dBodyID _robotBody = NULL;

static FILE * logfp;

// Dynamics model -------------------------------------------------

Dynamics::vehicle_params_t VPARAMS = {

    // Estimated
    2.E-06, // d drag cofficient [T=d*w^2]

    // https://www.dji.com/phantom-4/info
    1.380,  // m mass [kg]

    // Estimated
    2,      // Ix [kg*m^2] 
    2,      // Iy [kg*m^2] 
    3,      // Iz [kg*m^2] 
    38E-04, // Jr prop inertial [kg*m^2] 
    15000,  // maxrpm

    20      // maxspeed [m/s]
};

FixedPitchDynamics::fixed_pitch_params_t FPARAMS = {
    5.E-06, // b thrust coefficient [F=b*w^2]
    0.350   // l arm length [m]
};

static auto _dynamics = QuadXBFDynamics(VPARAMS, FPARAMS);

static const double DT = 0.01;

// Physics plug-in implementation ---------------------------------

DLLEXPORT void webots_physics_init() 
{
    logfp = fopen(LOGFILE, "w");

    // init global variables
    _robotBody = dWebotsGetBodyFromDEF(ROBOT_NAME);

    // disable gravity for the custom: buoyancy counteract gravity.
    dBodySetGravityMode(_robotBody, 0);

    double rotation[3] = {0, 0, 0};
    _dynamics.init(rotation);
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

    // If we have enough motor values, run the dynamics
    if (size == 4 * sizeof(float)) {

        _dynamics.update(motorvals, 0.01, logfp);

        dBodySetPosition(_robotBody, -1, 1, _dynamics.getStateZ());

        // auto pos = dBodyGetPosition(_robotBody);

        dMatrix3 rot = {0, -_dynamics.getStatePsi(), 0};
        dBodySetRotation(_robotBody, rot);
    }
}

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) {
    // We don't want to handle collisions
    return 0;
}

DLLEXPORT void webots_physics_cleanup() {
}
