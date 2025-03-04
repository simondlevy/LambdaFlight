/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

typedef struct {

    float thrust;  // positve upward
    float roll;    // positive roll right
    float pitch;   // positive nose down
    float yaw;     // positive nose right

} demands_t;

typedef struct {

    float x;
    float y;
    float z;

} axis3_t;


// From Eqn. (11) in Bouabdallah,  Murrieri, Siegwart (2004). 
// We use ENU coordinates based on 
// https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system
// Position in meters, velocity in meters/second, angles in degrees,
// angular velocity in degrees/second.
typedef struct {

    float dx;      // positive forward
    float dy;      // positive leftward
    float z;       // positive upward
    float dz;      // positive upward
    float phi;     // positive roll right
    float dphi;    // positive roll right
    float theta;   // positive nose up
    float dtheta;  // positive nose up (opposite of gyro Y)
    float psi;     // positive nose left
    float dpsi;    // positive nose left

    float z_dz;    // combination of Z and DZ for Python client

} vehicleState_t;

typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

typedef struct flowMeasurement_s {
    float dpixelx;  // Accumulated pixel count x
    float dpixely;  // Accumulated pixel count y
    float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;

typedef struct
{
  Axis3f gyro; // deg/s, for legacy reasons
} gyroscopeMeasurement_t;

typedef struct
{
  Axis3f acc; // Gs, for legacy reasons
} accelerationMeasurement_t;

typedef enum {
    MeasurementTypeRange,
    MeasurementTypeFlow,
    MeasurementTypeGyroscope,
    MeasurementTypeAcceleration,
} MeasurementType;

typedef struct {

    MeasurementType type;

    union {

        float rangefinder_distance;
        flowMeasurement_t flow;
        gyroscopeMeasurement_t gyroscope;
        accelerationMeasurement_t acceleration;
    } data;

} measurement_t;


//////////////////////////////////////////////////////////////////////////////

typedef void (*openLoopFun_t)(
        demands_t & demands, uint32_t & timestamp, bool & inHoverMode);

typedef union {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis[3];
} Axis3i16;

struct vec3_s {
    uint32_t timestamp; // Timestamp when the data was computed
    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;

typedef struct quaternion_s {
    union {
        struct {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

/* vector */
#define vec3d_size 3
typedef float vec3d[vec3d_size];
typedef float mat3d[vec3d_size][vec3d_size];

typedef struct vec3_s vector_t;

typedef enum {
    MeasurementSourceLocationService  = 0,
} measurementSource_t;

typedef struct sensorData_s {
    Axis3f acc;               // Gs
    Axis3f gyro;              // deg/s
    Axis3f mag;               // gauss
    uint64_t interruptTimestamp;
} sensorData_t;
