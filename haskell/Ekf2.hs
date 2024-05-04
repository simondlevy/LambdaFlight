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
ekfStep = qw where

  isFlying = true -- XXX

  shouldPredict = stream_now_msec >= _nextPredictionMsec;

  dt = 0.005 -- getDt stream_now_msec lastPredictionMsec

  gyro_sample_x = stream_gyro_x * degrees_to_radians
  gyro_sample_y = stream_gyro_y * degrees_to_radians
  gyro_sample_z = stream_gyro_z * degrees_to_radians

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

  qw = tmpq0 / norm
  qx = tmpq1 / norm
  qy = tmpq2 / norm
  qz = tmpq3 / norm

  lastPredictionMsec = if _lastPredictionMsec == 0  || shouldPredict
                        then stream_now_msec 
                        else _lastPredictionMsec

  nextPredictionMsec = if _nextPredictionMsec == 0 
                        then stream_now_msec 
                        else if stream_now_msec >= _nextPredictionMsec
                        then stream_now_msec + prediction_update_interval_msec
                        else _nextPredictionMsec

   -- Internal state, represented as streams ----------------------------------

  _nextPredictionMsec = [0] ++ nextPredictionMsec
  _lastPredictionMsec = [0] ++ lastPredictionMsec

  _qw = [1] ++ qw
  _qx = [0] ++ qx
  _qy = [0] ++ qy
  _qz = [0] ++ qz
