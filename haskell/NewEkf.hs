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

mode_init              = 0 :: EkfMode
mode_predict           = 1 :: EkfMode
mode_finalize          = 2 :: EkfMode
mode_get_state         = 3 :: EkfMode
mode_update_with_gyro  = 4 :: EkfMode
mode_update_with_accel = 5 :: EkfMode
mode_update_with_range = 6 :: EkfMode
mode_update_with_flow  = 7 :: EkfMode

------------------------------------------------------------------------------


ekfMode :: EkfMode
ekfMode = extern "stream_ekfMode" Nothing

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

type EkfMatrix = Array 7 (Array 7 SFloat)

pinit :: EkfMatrix

qq = sqr stdev_initial_position_z 
dd = sqr stdev_initial_velocity 
ee = sqr stdev_initial_attituderoll_pitch
rr = sqr stdev_initial_attitude_yaw

pinit =  array [--  z   dx  dy  dz  e0  e1  e2
             array [qq,  0,  0,  0,  0,  0,  0], -- z
             array [0,  dd,  0,  0,  0,  0,  0], -- dx
             array [0,  0,  dd,  0,  0,  0,  0], -- dy
             array [0,  0,  0,  dd,  0,  0,  0], -- dz
             array [0,  0,  0,  0,  ee,  0,  0], -- e0
             array [0,  0,  0,  0,  0,  ee,  0], -- e1
             array [0,  0,  0,  0,  0,  0,  rr]  -- e2
             ] 

------------------------------------------------------------------------------

afinalize :: SFloat -> SFloat -> SFloat -> EkfMatrix

afinalize v0 v1 v2 = a where

  e0 = v0/2 
  e1 = v1/2 
  e2 = v2/2

  e0e0 =  1 - e1*e1/2 - e2*e2/2
  e0e1 =  e2 + e0*e1/2
  e0e2 = (-e1) + e0*e2/2

  e1e0 =  (-e2) + e0*e1/2
  e1e1 = 1 - e0*e0/2 - e2*e2/2
  e1e2 = e0 + e1*e2/2

  e2e0 = e1 + e0*e2/2
  e2e1 = (-e0) + e1*e2/2
  e2e2 = 1 - e0*e0/2 - e1*e1/2

  a = array [    --  z   dx  dy  dz  e0  e1  e2
             array [1 , 0,  0,  0,  0,    0,     0],    -- z
             array [0,  1 , 0,  0,  0,    0,     0],    -- dx
             array [0,  0,  1 , 0,  0,    0,     0],    -- dy
             array [0,  0,  0,  1 , 0,    0,     0],    -- dz
             array [0,  0,  0,  0,  e0e0, e0e1,  e0e2], -- e0
             array [0,  0,  0,  0,  e1e0, e1e1,  e1e2], -- e1
             array [0,  0,  0,  0,  e2e0, e2e1,  e2e2]  -- e2
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
    ezz :: SFloat
  , edx :: SFloat
  , edy :: SFloat
  , edz :: SFloat
  , ee0 :: SFloat
  , ee1 :: SFloat
  , ee2 :: SFloat
}

------------------------------------------------------------------------------

data VehicleState = VehicleState {
    vz :: SFloat
  , vdx :: SFloat
  , vdy :: SFloat
  , vdz :: SFloat
  , phi :: SFloat
  , theta :: SFloat
  , psi :: SFloat
}

------------------------------------------------------------------------------

data Axis3 = Axis3 {
    x :: SFloat
  , y :: SFloat
  , z :: SFloat
}

------------------------------------------------------------------------------

data SubSampler = SubSampler { 
    sample :: Axis3
  , sum    :: Axis3
  , count  :: SInt32
}

------------------------------------------------------------------------------

data Ekf = Ekf {
    p :: EkfMatrix
  , r :: Axis3
  , quat :: Quaternion
  , ekfState :: EkfState
  , gyroSubSampler :: SubSampler
  , accelSubSampler :: SubSampler
  , isUpdated :: SBool
  , lastPredictionMsec :: SInt32
  , lastProcessedNoiseUpdateMsec :: SInt32
}

------------------------------------------------------------------------------

updateQuatValue :: SBool -> SBool -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat

updateQuatValue shouldPredict isErrorSufficient tmpq norm init curr =
  if ekfMode == mode_init then init
  else if shouldPredict then tmpq / norm
  else if isErrorSufficient then tmpq / norm
  else curr

------------------------------------------------------------------------------

updateRotationValue :: SFloat -> SFloat -> SFloat -> SFloat

updateRotationValue init curr final = 

  if ekfMode == mode_init then init
  else if ekfMode == mode_finalize then final
  else curr

------------------------------------------------------------------------------

updateEkfValue :: SBool -> SFloat -> SFloat -> SFloat

updateEkfValue shouldPredict curr predicted = 

  if ekfMode == mode_init then 0 
  else if shouldPredict then predicted
  else curr

------------------------------------------------------------------------------

rotateQuat :: SFloat -> SFloat -> SFloat

rotateQuat val initVal = val' where

  keep = 1 - rollpitch_zero_reversion

  val' = (val * (if isFlying then 1 else keep)) +
      (if isFlying then 0 else rollpitch_zero_reversion * initVal)

------------------------------------------------------------------------------

isErrorLarge :: SFloat -> SBool 
isErrorLarge v = abs v > 0.1e-3

isErrorInBounds :: SFloat -> SBool 
isErrorInBounds v = abs v < 10

------------------------------------------------------------------------------

subSamplerInit = SubSampler sample sum 0 where

  sum = Axis3 0 0 0

  sample = Axis3 0 0 0

------------------------------------------------------------------------------

ekfInit :: Ekf

ekfInit = Ekf p r q s g a false nowMsec nowMsec where 

  p = pinit

  r = Axis3 0 0 1

  q = Quaternion 1 0 0 0

  s = EkfState 0 0 0 0 0 0 0

  g = subSamplerInit

  a = subSamplerInit

------------------------------------------------------------------------------

ekfPredict :: Ekf -> Ekf

ekfPredict ekf = ekf 

------------------------------------------------------------------------------

ekfFinalize :: Ekf -> Ekf

ekfFinalize ekf = ekf where

  ekfs = ekfState ekf

  v0 = ee0 ekfs
  v1 = ee1 ekfs
  v2 = ee2 ekfs

  angle = sqrt (v0*v0 + v1*v1 + v2*v2) + eps
  ca = cos $ angle / 2
  sa = sin $ angle / 2

  dqw = ca
  dqx = sa * v0 / angle
  dqy = sa * v1 / angle
  dqz = sa * v2 / angle

  q = quat ekf
  qw = qqw q
  qx = qqx q
  qy = qqy q
  qz = qqz q

  -- Rotate the quad's attitude by the delta quaternion vector computed above
  tmpq0 = dqw * qw - dqx * qx - dqy * qy - dqz * qz
  tmpq1 = dqx * qw + dqw * qx + dqz * qy - dqy * qz
  tmpq2 = dqy * qw - dqz * qx + dqw * qy + dqx * qz
  tmpq3 = dqz * qw + dqy * qx - dqx * qy + dqw * qz

  -- Normalize and store the result
  norm = sqrt (tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + eps

  isErrorSufficient  = 
                (isErrorLarge v0  || isErrorLarge v1  || isErrorLarge v2 ) &&
                isErrorInBounds v0  && isErrorInBounds v1  && isErrorInBounds v2

 
  {-- Rotate the covariance, since we've rotated the body
  
   This comes from a second order approximation to:
   Sigma_post = exps(-d) Sigma_pre exps(-d)'
              ~ (I + [[-d]] + [[-d]]^2 / 2) 
   Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
   where d is the attitude error expressed as Rodriges parameters, ie. 
   d = tan(|v|/2)*v/|v|
  
   As derived in "Covariance Correction Step for Kalman Filtering with an 
   Attitude" http://arc.aiaa.org/doi/abs/10.2514/1.G000848
  --}

  -- The attitude error vector (v0,v1,v2) is small,  so we use a first-order
  -- approximation to e0 = tan(|v0|/2)*v0/|v0|
  a = afinalize v0 v1 v2

------------------------------------------------------------------------------

step = (vz, vdx, vdy, vdz, phi, theta, psi) where

  vz = 0 :: SFloat
  vdx = 0 :: SFloat
  vdy = 0 :: SFloat
  vdz = 0 :: SFloat
  phi = 0 :: SFloat
  theta = 0 :: SFloat
  psi = 0 :: SFloat

  --------------------------------------------------------------------------

{--
  _qw = [1] ++ qw
  _qx = [1] ++ qx
  _qy = [1] ++ qy
  _qz = [1] ++ qz
 
  _rx = [0] ++ rx
  _ry = [0] ++ ry
  _rz = [0] ++ rz

  _zz = [0] ++ zz
  _dx = [0] ++ dx
  _dy = [0] ++ dy
  _dz = [0] ++ dz
  _e0 = [0] ++ e0
  _e1 = [0] ++ e1
  _e2 = [0] ++ e2

  _lastPredictionMsec = [0] ++ lastPredictionMsec
--}

------------------------------------------------------------------------------

spec = do

  let (vz, vdx, vdy, vdz, phi, theta, psi) = step

  trigger "setState" 
           (ekfMode == mode_get_state) 
           [arg vz, arg vdx, arg vdy, arg vdz, arg phi, arg theta, arg psi]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
