{--
  EKF algorithm
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds        #-}
{-# LANGUAGE RebindableSyntax #-}

module MinEkf where

import Language.Copilot hiding(atan2, (!!))
import Copilot.Compile.C99

import Linear.Matrix hiding(transpose)
import Data.List hiding((++), sum) -- gives us transpose

import State
import Utils

-- Constants -----------------------------------------------------------------

gravity_magnitude = 9.81 :: SFloat
degrees_to_radians = pi / 180 :: SFloat
radians_to_degrees = 180 / pi :: SFloat

-- Initial variances, uncertain of position, but know we're
-- stationary and roughly flat
stdev_initial_attitude_roll_pitch = 0.01
stdev_initial_attitude_yaw = 0.01

proc''_att = 0 :: SFloat
meas''_gyro = 0.1 :: SFloat -- radians per second

-- Bounds on the covariance, these shouldn't be hit, but sometimes are... why?
max_covariance = 100 :: SFloat
min_covariance = 1e-6 :: SFloat

-- Small number epsilon, to prevent dividing by zero
eps = 1e-6 :: SFloat

-- Reversion of pitch and roll to zero
rollpitch_zero_reversion = 0.001 :: SFloat

-- This is slower than the imu update rate of 1000hz
prediction_rate = 100 :: SInt32
prediction_update_interval_msec = (div 1000  prediction_rate) :: SInt32

-- Streams from C++ ----------------------------------------------------------

stream_now_msec :: SInt32
stream_now_msec = extern "stream_now_msec" Nothing

stream_gyro_x :: SFloat
stream_gyro_x = extern "stream_gyro_x" Nothing

stream_gyro_y :: SFloat
stream_gyro_y = extern "stream_gyro_y" Nothing

stream_gyro_z :: SFloat
stream_gyro_z = extern "stream_gyro_z" Nothing

stream_accel_x :: SFloat
stream_accel_x = extern "stream_accel_x" Nothing

stream_accel_y :: SFloat
stream_accel_y = extern "stream_accel_y" Nothing

stream_accel_z :: SFloat
stream_accel_z = extern "stream_accel_z" Nothing

-- Utilities ------------------------------------------------------------------

type Vector = [SFloat]
type Matrix = [Vector]
type Index = Int

(!) :: Matrix -> (Index, Index) -> SFloat
a ! (i, j) = (a !! i) !! j

getDt :: SInt32 -> SInt32 -> SFloat
getDt msec1 msec2 = (unsafeCast (msec1 - msec2)) / 1000

rotateQuat :: SFloat -> SFloat -> SBool -> SFloat
rotateQuat val initVal isFlying = val' where

    keep = 1 - rollpitch_zero_reversion

    val' = (val * (if isFlying then  1 else keep)) +
           (if isFlying then 0 else rollpitch_zero_reversion * initVal)

updateCovarianceMatrix :: Matrix -> Matrix
updateCovarianceMatrix p = p

isErrorLarge :: SFloat -> SBool
isErrorLarge v = abs v > 0.1e-3

isErrorInBounds :: SFloat -> SBool
isErrorInBounds v = abs v < 10
 
-- EKF function --------------------------------------------------------------

ekfStep :: SFloat

ekfStep = qw where

  isFlying = true

  ----------------------------------------------------------------------------

  shouldPredict = true

  dt = 1

  gyro_sample_x = stream_gyro_x * degrees_to_radians
  gyro_sample_y = stream_gyro_y * degrees_to_radians
  gyro_sample_z = stream_gyro_z * degrees_to_radians

  accel_sample_x = stream_accel_x * gravity_magnitude
  accel_sample_y = stream_accel_y * gravity_magnitude
  accel_sample_z = stream_accel_z * gravity_magnitude

  e0 = gyro_sample_x * dt / 2
  e1 = gyro_sample_y * dt / 2
  e2 = gyro_sample_z * dt / 2

  e0e0 =  1 - e1*e1/2 - e2*e2/2
  e0e1 =  e2 + e0 * e1/2
  e0e2 = -e1 + e0 * e2/2

  e1e0 = -e2 + e0*e1/2
  e1e1 =  1 - e0*e0/2 - e2*e2/2
  e1e2 =  e0 + e1*e2/2

  e2e0 =  e1 + e0*e2/2
  e2e1 = -e0 + e1*e2/2
  e2e2 = 1 - e0*e0/2 - e1*e1/2

  a = [ [e0e0, e0e1, e0e2], 
        [e1e0, e1e1, e1e2], 
        [e2e0, e2e1, e2e2]  ]

  -- Attitude update (rotate by gyroscope) done via quaternions.
  -- This is the gyroscope angular velocity integrated over the sample period.
  dtwx = dt * gyro_sample_x
  dtwy = dt * gyro_sample_y
  dtwz = dt * gyro_sample_z

  -- Compute the quaternion values in [w,x,y,z] order
  angle = sqrt (dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + eps
  ca = cos $ angle / 2
  sa = sin $ angle / 2
  dqw = ca
  dqx = sa * dtwx / angle
  dqy = sa * dtwy / angle
  dqz = sa * dtwz / angle

  -- Rotate the quad's attitude by the delta quaternion vector computed above
  tmpq0 = rotateQuat (dqw*qw - dqx*qx - dqy*qy - dqz*qz) 1 isFlying
  tmpq1 = rotateQuat (dqx*qw + dqw*qx + dqz*qy - dqy*qz) 0 isFlying
  tmpq2 = rotateQuat (dqy*qw - dqz*qx + dqw*qy + dqx*qz) 0 isFlying
  tmpq3 = rotateQuat (dqz*qw + dqy*qx - dqx*qy + dqw*qz) 0 isFlying

  -- Normalize and store the result
  norm = sqrt (tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + eps

  p = [[p00,  p01,  p02],
       [p10,  p11,  p12],
       [p20,  p21,  p22]]

  -- Update the covariance matrix
  apa = a !*! p !*! (transpose a)
  
  qw_pred = if shouldPredict then tmpq0 / norm else _qw
  qx_pred = if shouldPredict then tmpq1 / norm else _qx 
  qy_pred = if shouldPredict then tmpq2 / norm else _qy 
  qz_pred = if shouldPredict then tmpq3 / norm else _qz

  p00 = 0 
  p01 = 0 
  p02 = 0 
  p10 = 0 
  p11 = 0 
  p12 = 0 
  p20 = 0 
  p21 = 0 
  p22 = 0 

  qw = qw_pred
  qx = 0 
  qy = 0 
  qz = 0 

  r20 = 0
  r21 = 0
  r22 = 0

  _p00 = 0
  _p01 = 0
  _p02 = 0
  _p10 = 0
  _p11 = 0
  _p12 = 0
  _p20 = 0
  _p21 = 0
  _p22 = 0

  _qw = [1] ++ qw
  _qx = [0] ++ qx
  _qy = [0] ++ qy
  _qz = [0] ++ qz
