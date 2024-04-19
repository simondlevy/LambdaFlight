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

import Data.List(transpose)
import Linear.Matrix hiding(transpose)

import LinAlg
import State
import Utils

-- Constants -----------------------------------------------------------------

-- Quaternion used for initial orientation
qw_init = 1 :: SFloat
qx_init = 0 :: SFloat
qy_init = 0 :: SFloat
qz_init = 0 :: SFloat

gravity_magnitude = 9.81 :: SFloat
degrees_to_radians = pi / 180 :: SFloat

-- Initial variances, uncertain of position, but know we're
-- stationary and roughly flat
stdev_initial_attitude_roll_pitch = 0.01 :: SFloat
stdev_initial_attitude_yaw = 0.01 :: SFloat

proc_noise_att = 0 :: SFloat
meas_noise_gyro = 0.1 :: SFloat -- radians per second

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

-- Subsampler for gyro and accelerometer --------------------------------------

type Subsampler = (SFloat, SFloat, SFloat, SFloat, SFloat, SFloat, SInt32)

subsamplerFinalize :: SBool -> SFloat -> Subsampler -> Subsampler

subsamplerFinalize shouldPredict conversionFactor 
  (sum_x, sum_y, sum_z, sample_x, sample_y, sample_z, count) = 
  (0, 0, 0, sample_x', sample_y', sample_z', 0) where 

  isCountNonzero = count > 0

  count' = unsafeCast count :: SFloat

  shouldFinalize = shouldPredict && isCountNonzero

  sample_x' = if shouldFinalize 
              then sum_x * conversionFactor / count'
              else sample_x

  sample_y' = if shouldFinalize 
              then sum_y * conversionFactor / count'
              else sample_y

  sample_z' = if shouldFinalize 
              then sum_z * conversionFactor / count'
              else sample_z

-- Streams from C++ ----------------------------------------------------------

now_msec :: SInt32
now_msec = extern "stream_now_msec" Nothing

gyro_x :: SFloat
gyro_x = extern "stream_gyro_x" Nothing

gyro_y :: SFloat
gyro_y = extern "stream_gyro_y" Nothing

gyro_z :: SFloat
gyro_z = extern "stream_gyro_z" Nothing

-- Utilities ------------------------------------------------------------------

square :: SFloat -> SFloat
square x = x * x

getDt :: SInt32 -> SInt32 -> SFloat
getDt msec1 msec2 = (unsafeCast (msec1 - msec2)) / 1000

rotateQuat :: SFloat -> SFloat -> SBool -> SFloat
rotateQuat val initVal isFlying = val' where

    keep = 1 - rollpitch_zero_reversion

    val' = (val * (if isFlying then  1 else keep)) +
           (if isFlying then 0 else rollpitch_zero_reversion * initVal)


-- EKF function --------------------------------------------------------------

ekfStep :: State

ekfStep = State dx dy zz dz phi dphi theta dtheta psi dpsi where

  -- XXX ---------------------------------------------------------------------

  dx = 0
  dy = 0
  zz = 0
  dz = 0
  phi = 0
  dphi = 0
  theta = 0
  dtheta = 0
  psi = 0
  dpsi = 0

  gyro_sum_x = 0 :: SFloat
  gyro_sum_y = 0 :: SFloat
  gyro_sum_z = 0 :: SFloat
  gyro_sample_x = 0 :: SFloat
  gyro_sample_y = 0 :: SFloat
  gyro_sample_z = 0 :: SFloat
  gyro_count = 0 :: SInt32

  accel_sum_x = 0 :: SFloat
  accel_sum_y = 0 :: SFloat
  accel_sum_z = 0 :: SFloat
  accel_sample_x = 0 :: SFloat
  accel_sample_y = 0 :: SFloat
  accel_sample_z = 0 :: SFloat
  accel_count = 0 :: SInt32

  isFlying = true

  qw = 0
  qx = 0
  qy = 0
  qz = 0

  p = [ [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0] ]

  ----------------------------------------------------------------------------

  shouldPredict = now_msec > nextPredictionMsec

  dt = getDt now_msec lastPredictionMsec

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
  tmpq0 = rotateQuat (dqw*qw - dqx*qx - dqy*qy - dqz*qz) qw_init isFlying
  tmpq1 = rotateQuat (dqx*qw + dqw*qx + dqz*qy - dqy*qz) qx_init isFlying
  tmpq2 = rotateQuat (dqy*qw - dqz*qx + dqw*qy + dqx*qz) qy_init isFlying
  tmpq3 = rotateQuat (dqz*qw + dqy*qx - dqx*qy + dqw*qz) qz_init isFlying

  -- Normalize and store the result
  norm = sqrt (tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + eps

  -- Update the covariance matrix
  apa = a !*! p !*! (transpose a)
  
  lastPredictionMsec = if _lastPredictionMsec == 0  || shouldPredict
                        then now_msec 
                        else _lastPredictionMsec

  nextPredictionMsec = if _nextPredictionMsec == 0 
                        then now_msec 
                        else if now_msec > _nextPredictionMsec
                        then now_msec + prediction_update_interval_msec
                        else _nextPredictionMsec

  (gyro_sum_x', gyro_sum_y', gyro_sum_z', 
   gyro_sample_x', gyro_sample_y', gyro_sample_z', gyro_count') = 
    subsamplerFinalize shouldPredict degrees_to_radians
    (gyro_sum_x, gyro_sum_y, gyro_sum_z, 
    gyro_sample_x, gyro_sample_y, gyro_sample_z, gyro_count)

  (accel_sum_x', accel_sum_y', accel_sum_z', 
   accel_sample_x', accel_sample_y', accel_sample_z', accel_count') = 
    subsamplerFinalize shouldPredict degrees_to_radians
    (accel_sum_x, accel_sum_y, accel_sum_z, 
    accel_sample_x, accel_sample_y, accel_sample_z, accel_count)

  -- Process noise is added after the return from the prediction step

  -- ====== PREDICTION STEP ======
  -- The prediction depends on whether we're on the
  -- ground, or in flight.  When flying, the
  -- accelerometer directly measures thrust (hence is
  -- useless to estimate body angle while flying)

  qw' = if shouldPredict then tmpq0 / norm else qw
  qx' = if shouldPredict then tmpq1 / norm else qx 
  qy' = if shouldPredict then tmpq2 / norm else qy 
  qz' = if shouldPredict then tmpq3 / norm else qz

  isUpdated = if shouldPredict then true else _isUpdated

  lastUpdateMsec = 0 -- XXX

  dt' = getDt now_msec lastUpdateMsec

  isDtPositive = dt' > 0

  noise = if isDtPositive 
          then square (meas_noise_gyro + dt' + proc_noise_att)
          else 0

  -- Internal state, represented as streams ----------------------------------

  _didInit = [False] ++ true

  _isUpdated = [False] ++ isUpdated

  _nextPredictionMsec = [0] ++ nextPredictionMsec

  _lastPredictionMsec = [0] ++ lastPredictionMsec

