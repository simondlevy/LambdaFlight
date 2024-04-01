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

-- Initial variances, uncertain of position, but know we're stationary and
-- roughly flat
stdev_initial_position_z          = 1.0  :: Float
stdev_initial_velocity            = 0.01 :: Float
stdev_initial_attituderoll_pitch = 0.01 :: Float
stdev_initial_attitude_yaw        = 0.01 :: Float

gravity_magnitude = 9.81 :: Float

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

sqr :: Float -> Float
sqr x = x * x

q = sqr stdev_initial_position_z 
d = sqr stdev_initial_velocity 
e = sqr stdev_initial_attituderoll_pitch
r = sqr stdev_initial_attitude_yaw

type EkfArray = Array 7 (Array 7 Float)

raw_pinit :: EkfArray

raw_pinit =  array [--  z   dx  dy  dz  e0  e1  e2
                 array [q,  0,  0,  0,  0,  0,  0], -- z
                 array [0,  d,  0,  0,  0,  0,  0], -- dx
                 array [0,  0,  d,  0,  0,  0,  0], -- dy
                 array [0,  0,  0,  d,  0,  0,  0], -- dz
                 array [0,  0,  0,  0,  e,  0,  0], -- e0
                 array [0,  0,  0,  0,  0,  e,  0], -- e1
                 array [0,  0,  0,  0,  0,  0,  r]  -- e2
             ] 

type SEkfArray = Stream EkfArray

pinit :: SEkfArray

pinit = [ raw_pinit ] ++ pinit

------------------------------------------------------------------------------

data Quat = Quat {
    qw :: SFloat
  , qx :: SFloat
  , qy :: SFloat
  , qz :: SFloat
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

data Axis3fSubSampler = Axis3fSubSampler { 
    sample :: Axis3f
  , sum    :: Axis3f
  , count  :: SInt32
}

subsampler_init = Axis3fSubSampler (Axis3f 0 0 0) (Axis3f 0 0 0) 0

subsampler_finalize :: SBool -> (SFloat -> SFloat) -> Axis3fSubSampler -> 
  Axis3fSubSampler

subsampler_finalize shouldPredict converter subsamp = subsamp' where

  cnt = unsafeCast (count subsamp) :: SFloat

  isCountNonzero = cnt > 0

  shouldFinalize = shouldPredict && isCountNonzero
 
  samp = sample subsamp

  ssum = sum subsamp

  x' = if shouldFinalize then (converter (x ssum)) / cnt else (x samp)
  y' = if shouldFinalize then (converter (y ssum)) / cnt else (y samp)
  z' = if shouldFinalize then (converter (z ssum)) / cnt else (z samp)


  subsamp' = Axis3fSubSampler (Axis3f x' y' z') (Axis3f 0 0 0) 0

------------------------------------------------------------------------------

-- init :: (SEkfArray, (SFloat, SFloat, SFloat, SFloat), SAxis3fSubSampler)
init :: (SEkfArray, (SFloat, SFloat, SFloat, SFloat))

init = (pinit, (1, 0, 0, 0) )

------------------------------------------------------------------------------

-----------------------------------------------------------------------------

predict :: SInt32 ->
           SFloat -> SFloat -> SFloat -> 
           SFloat -> SFloat -> SFloat -> SFloat ->
           (SFloat, SFloat, SFloat, SFloat, SFloat, SFloat, SFloat) 

predict lastPredictionMsec dx dy dz qw qx qy qz = 
  (dx', dy', dz', qw', qx', qy', qz') where

  shouldPredict = nowMsec >= nextPredictionMsec

--  (gx', gy', gz', gxsum', gysum', gzsum', gcount') = 
--    axis3fSubSamplerFinalize shouldPredict gx gy gz gxsum gysum gzsum gcount deg2rad

  dx' = dx
  dy' = dy
  dz' = dz

  qw' = qw
  qx' = qx
  qy' = qy
  qz' = qz

  dmsec = (unsafeCast $ nowMsec - lastPredictionMsec) :: SFloat

  dt = dmsec / 1000.0


{--
  axis3fSubSamplerFinalize(&_accSubSampler, shouldPredict)

  axis3fSubSamplerFinalize(&_gyroSubSampler, shouldPredict)

  const Axis3f * acc = &_accSubSampler.subSample 
  const Axis3f * gyro = &_gyroSubSampler.subSample 


  e0 = gx*dt/2
  e1 = gy*dt/2
  e2 = gz*dt/2

  -- altitude from body-frame velocity
  zdx = r20*dt
  zdy = r21*dt
  zdz = r22*dt

  -- altitude from attitude error
  ze0 = (dy*r22 - dz*r21)*dt
  ze1 = (- dx*r22 + dz*r20)*dt
  ze2 = (dx*r21 - dy*r20)*dt

  -- body-frame velocity from body-frame velocity
  dxdx = 1 --drag negligible
  dydx =-gz*dt
  dzdx = gy*dt

  dxdy = gz*dt
  dydy = 1 --drag negligible
  dzdy =-gx*dt

  dxdz =-gy*dt
  dydz = gx*dt
  dzdz = 1 --drag negligible

  -- body-frame velocity from attitude error
  dxe0 =  0
  dye0 = -gravity_magnitude*r22*dt
  dze0 =  gravity_magnitude*r21*dt

  dxe1 =  gravity_magnitude*r22*dt
  dye1 =  0
  dze1 = -gravity_magnitude*r20*dt

  dxe2 = -gravity_magnitude*r21*dt
  dye2 =  gravity_magnitude*r20*dt
  dze2 =  0

  e0e0 =  1 - e1*e1/2 - e2*e2/2
  e0e1 =  e2 + e0*e1/2
  e0e2 = -e1 + e0*e2/2

  e1e0 = -e2 + e0*e1/2
  e1e1 =  1 - e0*e0/2 - e2*e2/2
  e1e2 =  e0 + e1*e2/2

  e2e0 =  e1 + e0*e2/2
  e2e1 = -e0 + e1*e2/2
  e2e2 = 1 - e0*e0/2 - e1*e1/2
--}

------------------------------------------------------------------------------

step :: (SFloat, SFloat, SFloat, SFloat, SFloat, SFloat) 

step = (dx, dy, dz, phi, theta, psi) where

   is_init = ekfMode == mode_init

   is_predict = ekfMode == mode_predict

   (dx', dy', dz', qw', qx', qy', qz') = 
     predict _lastPredictionMsec _dx _dy _dz _qw _qx _qy _qz

   gyroSubsamplerX = (if is_init then 0 else _gyroSubsamplerX) :: SFloat
   gyroSubsamplerY = (if is_init then 0 else _gyroSubsamplerY) :: SFloat
   gyroSubsamplerZ = (if is_init then 0 else _gyroSubsamplerZ) :: SFloat

   accelSubsamplerX = (if is_init then 0 else _accelSubsamplerX) :: SFloat
   accelSubsamplerY = (if is_init then 0 else _accelSubsamplerY) :: SFloat
   accelSubsamplerZ = (if is_init then 0 else _accelSubsamplerZ) :: SFloat

   pmat = if is_init then pinit else _pmat

   dx = if is_init then 0 else if is_predict then dx' else _dx

   dy = if is_init then 0 else if is_predict then dy' else _dy

   dz = if is_init then 0 
        else if is_predict then dz' 
        else r20 * _dx + r21 * _dy + r22 * _dz

   z = (if is_init then 0 else _z) :: SFloat

   qw = if is_init then 1 else if is_predict then qw' else _qw
   qx = if is_init then 0 else if is_predict then qx' else _qx
   qy = if is_init then 0 else if is_predict then qy' else _qy
   qz = if is_init then 0 else if is_predict then qz' else _qz

   -- Set the is_initial rotation matrix to the identity. This only affects  the
   -- first prediction step, since in the finalization, after shifting 
   -- attitude errors into the attitude state, the rotation matrix is updated.
   r20 = if is_init then 0 else _r20
   r21 = if is_init then 0 else _r21
   r22 = if is_init then 1 else _r22

   isUpdated = if is_init then false else _isUpdated

   lastPredictionMsec = (if is_init then nowMsec else _lastPredictionMsec) :: SInt32

   lastProcessNoiseUpdateMsec = 
     (if is_init then nowMsec else _lastProcessNoiseUpdateMsec) :: SInt32

   phi = rad2deg $ atan2 (2 * (qy*qz + qw*qx)) (qw*qw - qx*qx - qy*qy + qz*qz)

   -- Negate for ENU
   theta = -(rad2deg $ asin ((-2) * (qx*qz - qw*qy)))

   psi = rad2deg $ atan2 (2 * (qx*qy + qw*qz)) (qw*qw + qx*qx - qy*qy - qz*qz)

   _pmat = [raw_pinit] ++ pmat

   -- EKF state
   _dx = [0] ++ dx
   _dy = [0] ++ dy
   _dz = [0] ++ dz
   _z = [0] ++ z

   -- Quaternion
   _qw = [0] ++ qw
   _qx = [0] ++ qx
   _qy = [0] ++ qy
   _qz = [0] ++ qz

   -- Rotation vector
   _r20 = [0] ++ r20
   _r21 = [0] ++ r21
   _r22 = [1] ++ r22

   -- Gyro subsampler
   _gyroSubsamplerX = [0] ++ gyroSubsamplerX 
   _gyroSubsamplerY = [0] ++ gyroSubsamplerY 
   _gyroSubsamplerZ = [0] ++ gyroSubsamplerZ 

   -- Accel subsampler
   _accelSubsamplerX = [0] ++ accelSubsamplerX 
   _accelSubsamplerY = [0] ++ accelSubsamplerY 
   _accelSubsamplerZ = [0] ++ accelSubsamplerZ 

   _isUpdated = [False] ++ isUpdated
   _lastPredictionMsec = [0] ++ lastPredictionMsec
   _lastProcessNoiseUpdateMsec = [0] ++ lastProcessNoiseUpdateMsec

------------------------------------------------------------------------------

spec = do

  let (dx, dy, dz, phi, theta, psi) = step

  trigger "setState" 
           (ekfMode == mode_get_state) 
           [arg dx, arg dy, arg dz, arg phi, arg theta, arg psi]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
