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

module Ekf where

import Language.Copilot
import Copilot.Compile.C99

import State
import Utils

-- Constants -----------------------------------------------------------------

-- Quaternion used for initial orientation
qw_init = 1 :: SFloat
qx_init = 0 :: SFloat
qy_init = 0 :: SFloat
qz_init = 0 :: SFloat

-- Initial variances, uncertain of position, but know we're
-- stationary and roughly flat
stdev_initial_attitude_roll_pitch = 0.01 :: SFloat
stdev_initial_attitude_yaw = 0.01 :: SFloat

proc_noise_att = 0 :: SFloat
meas_noise_gyro = 0.1 :: SFloat -- radians per second

gravity_magnitude = 9.81 :: SFloat

-- Bounds on the covariance, these shouldn't be hit, but sometimes are... why?
max_covariance = 100 :: SFloat
min_covariance = 1e-6 :: SFloat

-- Small number epsilon, to prevent dividing by zero
eps = 1e-6 :: SFloat

-- Reversion of pitch and roll to zero
rollpitch_zero_reversion = 0.001 :: SFloat

-- This is slower than the imu update rate of 1000hz
prediction_rate = 100 :: SInt32
prediction_update_interval_ms = (div 1000  prediction_rate) :: SInt32

-- Streams from C++ ----------------------------------------------------------

now_msec :: SInt32
now_msec = extern "stream_now_msec" Nothing

gyro_x :: SFloat
gyro_x = extern "stream_gyro_x" Nothing

gyro_y :: SFloat
gyro_y = extern "stream_gyro_y" Nothing

gyro_z :: SFloat
gyro_z = extern "stream_gyro_z" Nothing

-- Helpers --------------------------------------------------------------------

getDt :: SInt32 -> SInt32 -> SFloat

getDt msec1 msec2 = (unsafeCast (msec1 - msec2)) / 1000

-- EKF function --------------------------------------------------------------

ekfStep :: State

ekfStep = State 0 0 0 0 0 0 0 0 0 0 where

  nextPredictionMsec = 
      if _nextPredictionMsec == 0 then now_msec else _nextPredictionMsec

  lastPredictionMsec = 
      if _lastPredictionMsec == 0 then now_msec else _lastPredictionMsec

  shouldPredict = now_msec > nextPredictionMsec

  dt = getDt now_msec lastPredictionMsec

  gyro_sample_x = 0 :: SFloat
  gyro_sample_y = 0 :: SFloat
  gyro_sample_z = 0 :: SFloat

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



  -- Internal state, represented as streams ----------------------------------

  _didInit = [False] ++ true

  _nextPredictionMsec = [0] ++ nextPredictionMsec

  _lastPredictionMsec = [0] ++ lastPredictionMsec

