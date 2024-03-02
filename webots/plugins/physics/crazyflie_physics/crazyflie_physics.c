/****************************************************************************

  crazyflie_physic -- A Crazyflie physics model for Webots.

  Copyright (C) 2024 Simon D. Levy

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

 ******************************************************************************/

#include <plugins/physics.h>

#include "crazyflie2b.h"
#include "crazyfliemodel.h"
#include "utils.h"

// constants
const char kRobotName[] = "blimp_lis";
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
}*/


DLLEXPORT void webots_physics_init() 
{
    // logfp = fopen("/home/levys/Desktop/log.txt", "w");

    gRobotBody = dWebotsGetBodyFromDEF(kRobotName);

    if (gRobotBody == NULL) {
        dWebotsConsolePrintf("!!! crazyflie_physics :: webots_physics_init :: " 
                "error : could not get body of robot.\r\n");
    }
    else {
        // disable gravity
        dBodySetGravityMode(gRobotBody, 0);
    }
}

DLLEXPORT void webots_physics_step() 
{
    // we return if we have no robot to actuate.
    if (gRobotBody == NULL) {
        return;
    }

    dBodySetPosition(gRobotBody, 0, 0, 0);
}

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) {
    // We don't want to handle collisions
    return 0;
}

DLLEXPORT void webots_physics_cleanup() {
}
