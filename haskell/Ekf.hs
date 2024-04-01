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

module Main where

import Language.Copilot hiding(atan2, sum)
import Copilot.Compile.C99

import Utils

-- Quaternion used for initial orientation
qw_init = 1 :: SFloat
qx_init = 0 :: SFloat
qy_init = 0 :: SFloat
qz_init = 0 :: SFloat

-- Initial variances, uncertain of position, but know we're stationary and
-- roughly flat
stdev_initial_position_z         = 1.0  :: SFloat
stdev_initial_velocity           = 0.01 :: SFloat
stdev_initial_attituderoll_pitch = 0.01 :: SFloat
stdev_initial_attitude_yaw       = 0.01 :: SFloat

--The reversion of pitch and roll to zero
rollpitch_zero_reversion = 0.001 :: SFloat

gravity_magnitude = 9.81 :: SFloat

-- Small number epsilon, to prevent dividing by zero
eps = 1e-6 :: SFloat

------------------------------------------------------------------------------

type EkfMode = SInt8

ekfMode :: EkfMode
ekfMode = extern "stream_ekfMode" Nothing

mode_init              = 0 :: EkfMode
mode_predict           = 1 :: EkfMode
mode_finalize          = 2 :: EkfMode
mode_get_state         = 3 :: EkfMode
mode_update_with_gyro  = 4 :: EkfMode
mode_update_with_accel = 5 :: EkfMode
mode_update_with_range = 6 :: EkfMode
mode_update_with_flow  = 7 :: EkfMode

nowMsec :: SInt32
nowMsec = extern "stream_nowMsec" Nothing

nextPredictionMsec :: SInt32
nextPredictionMsec = extern "stream_nextPredictionMsec" Nothing

isFlying :: SBool
isFlying = extern "stream_isFlying" Nothing

gx :: SFloat
gx = extern "stream_gx" Nothing

gyroY :: SFloat
gyroY = extern "stream_gyroY" Nothing

gyroZ :: SFloat
gyroZ = extern "stream_gyroZ" Nothing

accelX :: SFloat
accelX = extern "stream_accelX" Nothing

accelY :: SFloat
accelY = extern "stream_accelY" Nothing

accelZ :: SFloat
accelZ = extern "stream_accelZ" Nothing

------------------------------------------------------------------------------

sqr :: SFloat -> SFloat
sqr x = x * x

q = sqr stdev_initial_position_z 
d = sqr stdev_initial_velocity 
e = sqr stdev_initial_attituderoll_pitch
r = sqr stdev_initial_attitude_yaw

type EkfMatrix = Array 7 (Array 7 SFloat)

pinit :: EkfMatrix

pinit =  array [--  z   dx  dy  dz  e0  e1  e2
             array [q,  0,  0,  0,  0,  0,  0], -- z
             array [0,  d,  0,  0,  0,  0,  0], -- dx
             array [0,  0,  d,  0,  0,  0,  0], -- dy
             array [0,  0,  0,  d,  0,  0,  0], -- dz
             array [0,  0,  0,  0,  e,  0,  0], -- e0
             array [0,  0,  0,  0,  0,  e,  0], -- e1
             array [0,  0,  0,  0,  0,  0,  r]  -- e2
             ] 

------------------------------------------------------------------------------

data Quaternion = Quaternion {
    qqw :: SFloat
  , qqx :: SFloat
  , qqy :: SFloat
  , qqz :: SFloat
}

------------------------------------------------------------------------------

data EkfState = EkfState {
    zz :: SFloat
  , dx :: SFloat
  , dy :: SFloat
  , dz :: SFloat
  , e0 :: SFloat
  , e1 :: SFloat
  , e2 :: SFloat
}

------------------------------------------------------------------------------

data Axis3f = Axis3f {
    x :: SFloat
  , y :: SFloat
  , z :: SFloat
}

------------------------------------------------------------------------------

data SubSampler = SubSampler { 
    sample :: Axis3f
  , sum    :: Axis3f
  , count  :: SInt32
}

subsampler_init = SubSampler (Axis3f 0 0 0) (Axis3f 0 0 0) 0

subsampler_finalize :: SBool -> (SFloat -> SFloat) -> SubSampler -> 
  SubSampler

subsampler_finalize shouldPredict converter subsamp = subsamp' where

  cnt = unsafeCast (count subsamp) :: SFloat

  isCountNonzero = cnt > 0

  shouldFinalize = shouldPredict && isCountNonzero
 
  samp = sample subsamp

  ssum = sum subsamp

  x' = if shouldFinalize then (converter (x ssum)) / cnt else (x samp)
  y' = if shouldFinalize then (converter (y ssum)) / cnt else (y samp)
  z' = if shouldFinalize then (converter (z ssum)) / cnt else (z samp)

  subsamp' = SubSampler (Axis3f x' y' z') (Axis3f 0 0 0) 0


-----------------------------------------------------------------------------

predict :: SInt32 -> EkfState -> Quaternion -> Axis3f ->  SubSampler -> SubSampler ->
  (EkfState, Quaternion, SubSampler, SubSampler)

predict lastPredictionMsec ekfState quat r gyroSubSampler accelSubSampler = 
  (ekfState', quat', gyroSubSampler', accelSubSampler') where

  shouldPredict = nowMsec >= nextPredictionMsec

  gyroSubSampler' = subsampler_finalize shouldPredict deg2rad gyroSubSampler

  gyro = (sample gyroSubSampler')

  dmsec = (unsafeCast $ nowMsec - lastPredictionMsec) :: SFloat

  dt = dmsec / 1000.0

  e0 = (x gyro) * dt / 2
  e1 = (y gyro) * dt / 2
  e2 = (z gyro) * dt / 2

  -- altitude from body-frame velocity
  zdx = (x r) * dt
  zdy = (y r) * dt
  zdz = (z r) * dt

  -- altitude from attitude error
  ze0 = ((dy ekfState) * (z r) - (dz ekfState) * (y r)) * dt
  ze1 = (- (dx ekfState) * (z r) + (dz ekfState) * (x r)) * dt
  ze2 = ((dx ekfState) * (y r) - (dy ekfState) * (x r)) * dt

  -- body-frame velocity from body-frame velocity
  dxdx = 1 :: SFloat -- drag negligible
  dydx = -(z gyro) * dt
  dzdx =  (y gyro) *dt

  dxdy =  (z gyro) * dt
  dydy =  1 :: SFloat --drag negligible
  dzdy = -(x gyro) * dt

  dxdz = -(y gyro) * dt
  dydz =  (x gyro) * dt
  dzdz =  1 --drag negligible

  -- body-frame velocity from attitude error
  dxe0 =  0 :: SFloat
  dye0 = -gravity_magnitude * (z r) * dt
  dze0 =  gravity_magnitude * (y r) * dt

  dxe1 =  gravity_magnitude * (z r) * dt
  dye1 =  0 :: SFloat
  dze1 = -gravity_magnitude * (x r) * dt

  dxe2 = -gravity_magnitude * (y r) * dt
  dye2 =  gravity_magnitude * (x r) * dt
  dze2 =  0 :: SFloat

  e0e0 =  1 - e1 * e1/2 - e2 * e2/2
  e0e1 =  e2 + e0 * e1/2
  e0e2 = -e1 + e0 * e2/2

  e1e0 = (-e2) + e0 * e1/2
  e1e1 =  1 - e0 * e0 /2 - e2 * e2 /2
  e1e2 =  e0 + e1 * e2/2

  e2e0 =  e1 + e0 * e2/2
  e2e1 = (-e0) + e1 * e2/2
  e2e2 = 1 - e0 * e0/2 - e1 * e1/2

  a =  array [--    z   dx    dy    dz    e0    e1    e2
             array [0,  zdx,  zdy,  zdz,  ze0,  ze1,  ze2], -- z
             array [0, dxdx, dxdy, dxdz, dxe0, dxe1, dxe2], -- dx
             array [0, dydx, dydy, dydz, dye0, dye1, dye2], -- dy
             array [0, dzdx, dzdy, dzdz, dze0, dze1, dze2], -- dz
             array [0, 0,    0,    0,    e0e0, e0e1, e0e2], -- e0
             array [0, 0,    0,    0,    e1e0, e1e1, e1e2], -- e1
             array [0, 0,    0,    0,    e2e0, e2e1, e2e2]  -- e2
             ] :: EkfMatrix

  dt2 = dt * dt

  mss2g = \mss -> mss / gravity_magnitude
  
  accelSubSampler' = subsampler_finalize shouldPredict mss2g accelSubSampler

  accel = (sample accelSubSampler')

  -- Position updates in the body frame (will be rotated to inertial frame)
  -- thrust can only be produced in the body's Z direction
  dx' = (dx ekfState) * dt + (if isFlying then 0 else (x accel) * dt2 / 2)
  dy' = (dy ekfState) * dt + (if isFlying then 0 else (y accel) * dt2 / 2)
  dz' = (dz ekfState) * dt + (z accel)  * dt2 / 2 

  -- Keep previous time step's state for the update
  tmpSDX = (dx ekfState)
  tmpSDY = (dy ekfState)
  tmpSDZ = (dz ekfState)

  accx = if isFlying then 0 else x accel
  accy = if isFlying then 0 else y accel
 
  -- Attitude update (rotate by gyroscope), we do this in quaternions:
  -- this is the gyroscope angular velocity integrated over the sample period
  dtwx = dt * (x gyro)
  dtwy = dt * (y gyro)
  dtwz = dt * (z gyro)

  -- Compute the quaternion values in [w,x,y,z] order
  angle = (sqrt (dtwx * dtwx + dtwy * dtwy + dtwz * dtwz)) + eps
  ca = cos (angle / 2)
  sa = sin (angle / 2)
  dqw = ca
  dqx = sa * dtwx / angle
  dqy = sa * dtwy / angle
  dqz = sa * dtwz / angle

  qw = qqw quat
  qx = qqx quat
  qy = qqy quat
  qz = qqz quat

  -- Rotate the quad's attitude by the delta quaternion vector computed above
  tmpq0 = rotateQuat (dqw * qw - dqx * qx - dqy * qy - dqz * qz) qw_init isFlying
  tmpq1 = rotateQuat (dqx * qw + dqw * qx + dqz * qy - dqy * qz) qx_init isFlying
  tmpq2 = rotateQuat (dqy * qw - dqz * qx + dqw * qy + dqx * qz) qy_init isFlying
  tmpq3 = rotateQuat (dqz * qw + dqy * qx - dqx * qy + dqw * qz) qz_init isFlying

  -- Normalize and store the result
  norm = sqrt (tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + eps

 
  ekfState' = ekfState

  quat' = quat


rotateQuat :: SFloat -> SFloat -> SBool -> SFloat

rotateQuat val initVal isFlying = val' where

    keep = 1 - rollpitch_zero_reversion

    val' = (val * (if isFlying then 1 else keep)) +
        (if isFlying then 0 else rollpitch_zero_reversion * initVal)



------------------------------------------------------------------------------

step :: (SFloat, SFloat, SFloat, SFloat, SFloat, SFloat, SFloat) 

step = (z, dx, dy, dz, phi, theta, psi) where

   is_init = ekfMode == mode_init

   is_predict = ekfMode == mode_predict

   z = 0
   dx = 0
   dy = 0
   dz = 0

   ekfState = EkfState 0 0 0 0 0 0 0

   quat = Quaternion _qw _qx _qy _qz

   r = Axis3f 0 0 0

   gyroSubSampler = subsampler_init
   accelSubSampler = subsampler_init

   (ekfState', quat', gyroSubSampler', accelSubSampler') = 
     predict lastPredictionMsec ekfState quat r gyroSubSampler accelSubSampler

   qw = if is_init then 1 else if is_predict then (qqw quat') else _qw
   qx = if is_init then 0 else if is_predict then (qqx quat') else _qx
   qy = if is_init then 0 else if is_predict then (qqy quat') else _qy
   qz = if is_init then 0 else if is_predict then (qqz quat') else _qz

   -- Set the is_initial rotation matrix to the identity. This only affects  the
   -- first prediction step, since in the finalization, after shifting 
   -- attitude errors into the attitude state, the rotation matrix is updated.
   r20 = (if is_init then 0 else _r20) :: SFloat
   r21 = (if is_init then 0 else _r21) :: SFloat
   r22 = (if is_init then 1 else _r22) :: SFloat

   isUpdated = if is_init then false else _isUpdated

   lastPredictionMsec = (if is_init then nowMsec else _lastPredictionMsec) :: SInt32

   lastProcessNoiseUpdateMsec = 
     (if is_init then nowMsec else _lastProcessNoiseUpdateMsec) :: SInt32

   phi = rad2deg $ atan2 (2 * (qy*qz + qw*qx)) (qw*qw - qx*qx - qy*qy + qz*qz)

   -- Negate for ENU
   theta = -(rad2deg $ asin ((-2) * (qx*qz - qw*qy)))

   psi = rad2deg $ atan2 (2 * (qx*qy + qw*qz)) (qw*qw + qx*qx - qy*qy - qz*qz)

   -- Quaternion
   _qw = [0] ++ qw
   _qx = [0] ++ qx
   _qy = [0] ++ qy
   _qz = [0] ++ qz

   -- Rotation vector
   _r20 = [0] ++ r20
   _r21 = [0] ++ r21
   _r22 = [1] ++ r22

   _isUpdated = [False] ++ isUpdated
   _lastPredictionMsec = [0] ++ lastPredictionMsec
   _lastProcessNoiseUpdateMsec = [0] ++ lastProcessNoiseUpdateMsec

------------------------------------------------------------------------------

spec = do

  let (z, dx, dy, dz, phi, theta, psi) = step

  trigger "setState" 
           (ekfMode == mode_get_state) 
           [arg z, arg dx, arg dy, arg dz, arg phi, arg theta, arg psi]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
