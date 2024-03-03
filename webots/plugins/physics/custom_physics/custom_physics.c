/****************************************************************************

  custom_physics -- A custom physics model for Webots.

  Copyright (C) 2006 Laboratory of Intelligent Systems, EPFL, Lausanne
Authors:    Alexis Guanella            guanella@ini.phys.ethz.ch
Antoine Beyeler            antoine.beyeler@epfl.ch
Jean-Christophe Zufferey   jean-christophe.zufferey@epfl.ch
Dario Floreano             dario.floreano@epfl.ch
Web: http://lis.epfl.ch

The authors of any publication arising from research using this software are
kindly requested to add the following reference:

Zufferey, J.C., Guanella, A., Beyeler, A., Floreano, D. (2006) Flying over
the Reality Gap: From Simulated to Real Indoor Airships. Autonomous Robots,
Springer US.

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
/*------------------------------------------------------------------------------

Author:		Antoine Beyeler (ab)

------------------------------------------------------------------------------*/

#include <plugins/physics.h>

#include "custom2b.h"
#include "custommodel.h"
#include "utils.h"

// constants
const char kRobotName[] = "custom_lis";
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
    if (gRobotBody == NULL)
        return;

    dBodySetPosition(gRobotBody, -1, 1, 1);
}

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) {
    // We don't want to handle collisions
    return 0;
}

DLLEXPORT void webots_physics_cleanup() {
}
