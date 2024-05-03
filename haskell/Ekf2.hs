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

module Ekf2 where

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

proc_att = 0 :: SFloat
meas_gyro = 0.1 :: SFloat -- radians per second

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

isErrorLarge :: SFloat -> SBool
isErrorLarge v = abs v > 0.1e-3

isErrorInBounds :: SFloat -> SBool
isErrorInBounds v = abs v < 10
 
-- EKF function --------------------------------------------------------------

--ekfStep :: (SFloat, SFloat, SFloat, SFloat)
--ekfStep = (qw, qx, qy, qz) where

ekfStep :: SFloat
ekfStep = tmpq0 / norm where

  isFlying = true -- XXX

  shouldPredict = stream_now_msec >= _nextPredictionMsec

  dt = getDt stream_now_msec lastPredictionMsec

  gyro_sample_x = stream_gyro_x * degrees_to_radians
  gyro_sample_y = stream_gyro_y * degrees_to_radians
  gyro_sample_z = stream_gyro_z * degrees_to_radians

  e0 = gyro_sample_x * dt / 2
  e1 = gyro_sample_y * dt / 2
  e2 = gyro_sample_z * dt / 2

  dtwx = dt * gyro_sample_x
  dtwy = dt * gyro_sample_y
  dtwz = dt * gyro_sample_z

  angle = sqrt (dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + eps
  ca = cos $ angle / 2
  sa = sin $ angle / 2
  dqw = ca
  dqx = sa * dtwx / angle
  dqy = sa * dtwy / angle
  dqz = sa * dtwz / angle

  tmpq0 = rotateQuat (dqw*_qw - dqx*_qx - dqy*_qy - dqz*_qz) 1 isFlying
  tmpq1 = rotateQuat (dqx*_qw + dqw*_qx + dqz*_qy - dqy*_qz) 0 isFlying
  tmpq2 = rotateQuat (dqy*_qw - dqz*_qx + dqw*_qy + dqx*_qz) 0 isFlying
  tmpq3 = rotateQuat (dqz*_qw + dqy*_qx - dqx*_qy + dqw*_qz) 0 isFlying

  norm = sqrt (tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + eps

  lastPredictionMsec = if _lastPredictionMsec == 0  || shouldPredict
                        then stream_now_msec 
                        else _lastPredictionMsec

  nextPredictionMsec = if _nextPredictionMsec == 0 
                        then stream_now_msec 
                        else if stream_now_msec >= _nextPredictionMsec
                        then stream_now_msec + prediction_update_interval_msec
                        else _nextPredictionMsec

  qw_pred = if shouldPredict then tmpq0 / norm else _qw
  qx_pred = if shouldPredict then tmpq1 / norm else _qx 
  qy_pred = if shouldPredict then tmpq2 / norm else _qy 
  qz_pred = if shouldPredict then tmpq3 / norm else _qz

  dt' = getDt stream_now_msec lastUpdateMsec

  isDtPositive = dt' > 0

  noise = if isDtPositive then (meas_gyro + dt' + proc_att) ** 2 else 0

  lastUpdateMsec = if _lastUpdateMsec == 0 || isDtPositive 
                   then  stream_now_msec 
                   else _lastUpdateMsec

  -- Incorporate the attitude error (Kalman filter state) with the attitude
  v0 = 0 -- e0
  v1 = 0 -- e1
  v2 = 0 -- e2

  newangle = sqrt (v0*v0 + v1*v1 + v2*v2) + eps
  newca = cos(newangle / 2)
  newsa = sin(newangle / 2)

  newdqw = newca
  newdqx = newsa * v0 / newangle
  newdqy = newsa * v1 / newangle
  newdqz = newsa * v2 / newangle
            
  -- Rotate the quad's attitude by the delta quaternion vector computed above
  newtmpq0 = newdqw * _qw - newdqx * _qx - newdqy * _qy - newdqz * _qz
  newtmpq1 = newdqx * _qw + newdqw * _qx + newdqz * _qy - newdqy * _qz
  newtmpq2 = newdqy * _qw - newdqz * _qx + newdqw * _qy + newdqx * _qz
  newtmpq3 = newdqz * _qw + newdqy * _qx - newdqx * _qy + newdqw * _qz

   -- Normalize and store the result
  newnorm = sqrt (newtmpq0 * newtmpq0 + newtmpq1 * newtmpq1 + 
                  newtmpq2 * newtmpq2 + newtmpq3 * newtmpq3) + eps

  -- The attitude error vector (v0,v1,v2) is small,  so we use a first-order
  -- approximation to e0 = tan(|v0|/2)*v0/|v0|
  newe0 = v0 / 2 
  newe1 = v1 / 2 
  newe2 = v2 / 2

  newe0e0 =  1 - newe1*newe1/2 - newe2*newe2/2
  newe0e1 =  newe2 + newe0*newe1/2
  newe0e2 = -newe1 + newe0*newe2/2

  newe1e0 =  -newe2 + newe0*newe1/2
  newe1e1 = 1 - newe0*newe0/2 - newe2*newe2/2
  newe1e2 = newe0 + newe1*newe2/2

  newe2e0 = newe1 + newe0*newe2/2
  newe2e1 = -newe0 + newe1*newe2/2
  newe2e2 = 1 - newe0*newe0/2 - newe1*newe1/2

  isErrorSufficient  = 
    (isErrorLarge v0 || isErrorLarge v1 || isErrorLarge v2) &&
    isErrorInBounds v0 && isErrorInBounds v1 && isErrorInBounds v2

  -- Matrix to rotate the attitude covariances once updated
  newa = [ [newe0e0, newe0e1, newe0e2],
           [newe1e0, newe1e1, newe1e2],
           [newe2e0, newe2e1, newe2e2] ]

  qw = if isErrorSufficient then newtmpq0 / newnorm else qw_pred
  qx = if isErrorSufficient then newtmpq1 / newnorm else qx_pred
  qy = if isErrorSufficient then newtmpq2 / newnorm else qy_pred
  qz = if isErrorSufficient then newtmpq3 / newnorm else qz_pred

   -- Internal state, represented as streams ----------------------------------

  _qw = [1] ++ qw
  _qx = [0] ++ qx
  _qy = [0] ++ qy
  _qz = [0] ++ qz

  _didInit = [False] ++ true
  _nextPredictionMsec = [0] ++ nextPredictionMsec
  _lastPredictionMsec = [0] ++ lastPredictionMsec
  _lastUpdateMsec = [0] ++ lastUpdateMsec
