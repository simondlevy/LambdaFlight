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

ekfStep = dx where

  -- XXX ---------------------------------------------------------------------

  dx = 0
  dy = 0
  zz = 0
  dz = 0

  isFlying = true

  ----------------------------------------------------------------------------

  shouldPredict = stream_now_msec > nextPredictionMsec

  dt = getDt stream_now_msec lastPredictionMsec

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
  
  lastPredictionMsec = if _lastPredictionMsec == 0  || shouldPredict
                        then stream_now_msec 
                        else _lastPredictionMsec

  nextPredictionMsec = if _nextPredictionMsec == 0 
                        then stream_now_msec 
                        else if stream_now_msec > _nextPredictionMsec
                        then stream_now_msec + prediction_update_interval_msec
                        else _nextPredictionMsec

  -- Process noise is added after the return from the prediction step

  -- ====== PREDICTION STEP ======
  -- The prediction depends on whether we're on the
  -- ground, or in flight.  When flying, the
  -- accelerometer directly measures thrust (hence is
  -- useless to estimate body angle while flying)

  qw_pred = if shouldPredict then tmpq0 / norm else _qw
  qx_pred = if shouldPredict then tmpq1 / norm else _qx 
  qy_pred = if shouldPredict then tmpq2 / norm else _qy 
  qz_pred = if shouldPredict then tmpq3 / norm else _qz

  dt' = getDt stream_now_msec lastUpdateMsec

  isDtPositive = dt' > 0

  noise = if isDtPositive 
          then (meas''_gyro + dt' + proc''_att) ** 2
          else 0

  p00_pred = if shouldPredict then apa!(0,0) + noise else p00
  p01_pred = if shouldPredict then apa!(0,1) + noise else p01
  p02_pred = if shouldPredict then apa!(0,2) + noise else p02
  p10_pred = if shouldPredict then apa!(1,0) + noise else p10
  p11_pred = if shouldPredict then apa!(1,1) + noise else p11
  p12_pred = if shouldPredict then apa!(1,2) + noise else p12
  p20_pred = if shouldPredict then apa!(2,0) + noise else p20
  p21_pred = if shouldPredict then apa!(2,1) + noise else p21
  p22_pred = if shouldPredict then apa!(2,2) + noise else p22

  p_pred = [[p00_pred,  p01_pred,  p02_pred],
            [p10_pred,  p11_pred,  p12_pred],
            [p20_pred,  p21_pred,  p22_pred]]

  p3 = updateCovarianceMatrix p_pred
 
  p00_noise = if isDtPositive then p3!(0,0) else p00_pred
  p01_noise = if isDtPositive then p3!(0,1) else p01_pred
  p02_noise = if isDtPositive then p3!(0,2) else p02_pred
  p10_noise = if isDtPositive then p3!(1,0) else p10_pred
  p11_noise = if isDtPositive then p3!(1,1) else p11_pred
  p12_noise = if isDtPositive then p3!(1,2) else p12_pred
  p20_noise = if isDtPositive then p3!(2,0) else p20_pred
  p21_noise = if isDtPositive then p3!(2,1) else p21_pred
  p22_noise = if isDtPositive then p3!(2,2) else p22_pred

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

  {--
    Rotate the covariance, since we've rotated the body
             
    This comes from a second order approximation to:
    Sigma_post = exps(-d) 
    Sigma_pre exps(-d)'
               ~ (I + [[-d]] + [[-d]]^2 / 2) 
    (I + [[-d]] + [[-d]]^2 / 2)'
    where d is the attitude error expressed as Rodriges parameters,
    ie.  d = tan(|v|/2)*v/|v|
    As derived in "Covariance Correction Step for Kalman Filtering
    with an Attitude"
    http://arc.aiaa.org/doi/abs/10.2514/1.G000848
  --}

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

  p4 = [ [p00_noise, p01_noise, p02_noise],
         [p10_noise, p11_noise, p12_noise],
         [p20_noise, p21_noise, p22_noise] ]

  newapa = newa !*! p4 !*! (transpose newa)

  p00_final = if isErrorSufficient then newapa!(0,0) else p00_noise
  p01_final = if isErrorSufficient then newapa!(0,1) else p01_noise
  p02_final = if isErrorSufficient then newapa!(0,2) else p02_noise
  p10_final = if isErrorSufficient then newapa!(1,0) else p10_noise
  p11_final = if isErrorSufficient then newapa!(1,1) else p11_noise
  p12_final = if isErrorSufficient then newapa!(1,2) else p12_noise
  p20_final = if isErrorSufficient then newapa!(2,0) else p20_noise
  p21_final = if isErrorSufficient then newapa!(2,1) else p21_noise
  p22_final = if isErrorSufficient then newapa!(2,2) else p22_noise

  p5 =  [ [p00_final, p01_final, p02_final],
          [p10_final, p11_final, p12_final],
          [p20_final, p21_final, p22_final] ]

  p6 = updateCovarianceMatrix p5

  p00 = if isErrorSufficient then p6!(0,0) else _p00
  p01 = if isErrorSufficient then p6!(0,1) else _p01
  p02 = if isErrorSufficient then p6!(0,2) else _p02
  p10 = if isErrorSufficient then p6!(1,0) else _p10
  p11 = if isErrorSufficient then p6!(1,1) else _p11
  p12 = if isErrorSufficient then p6!(1,2) else _p12
  p20 = if isErrorSufficient then p6!(2,0) else _p20
  p21 = if isErrorSufficient then p6!(2,1) else _p21
  p22 = if isErrorSufficient then p6!(2,2) else _p22

  qw = if isErrorSufficient then newtmpq0 / newnorm else qw_pred
  qx = if isErrorSufficient then newtmpq1 / newnorm else qx_pred
  qy = if isErrorSufficient then newtmpq2 / newnorm else qy_pred
  qz = if isErrorSufficient then newtmpq3 / newnorm else qz_pred

  -- Convert the new attitude to a rotation matrix, such  that we can rotate 
  -- body-frame velocity and acc
  r20 = if isErrorSufficient then  2 * qx * qz - 2 * qw * qy else _r20
  r21 = if isErrorSufficient then  2 * qy * qz + 2 * qw * qx else _r21
  r22 = if isErrorSufficient then qw * qw - qx * qx - qy * qy + qz * qz else _r22

  -- Get the vehicle state ---------------------------------------------------
  
  phi = radians_to_degrees * atan2 (2 * (qy*qz + qw*qx))
                                   (qw*qw - qx*qx - qy*qy + qz*qz)
 
  -- Negate for ENU
  theta = (-radians_to_degrees) * asin ((-2) * (_qx*_qz - _qw*_qy))

  psi = radians_to_degrees * atan2 (2 * (qx*qy + qw*qz))
                                   (qw*qw + qx*qx - qy*qy - qz*qz)

  -- Get angular velocities directly from gyro
  dphi =    stream_gyro_x
  dtheta = -stream_gyro_y -- negate for ENU
  dpsi =    stream_gyro_z

   -- Internal state, represented as streams ----------------------------------

  _didInit = [False] ++ true
  _nextPredictionMsec = [0] ++ nextPredictionMsec
  _lastPredictionMsec = [0] ++ lastPredictionMsec
  _lastUpdateMsec = [0] ++ lastUpdateMsec

  -- Covariance matrix entries
  _p00 = [stdev_initial_attitude_roll_pitch ** 2] ++ p00
  _p01 = [0] ++ p01
  _p02 = [0] ++ p01
  _p10 = [0] ++ p01
  _p11 = [stdev_initial_attitude_roll_pitch ** 2] ++ p11
  _p12 = [0] ++ p01
  _p20 = [0] ++ p01
  _p21 = [0] ++ p01
  _p22 = [stdev_initial_attitude_yaw ** 2] ++ p22

  -- Quaternion
  _qw = [1] ++ qw
  _qx = [0] ++ qx
  _qy = [0] ++ qy
  _qz = [0] ++ qz

  -- Third row (Z) of attitude as a rotation matrix (used by prediction, 
  -- updated by finalization)
  _r20 = [0] ++ r20
  _r21 = [0] ++ r21
  _r22 = [1] ++ r22

