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

qq = sqr stdev_initial_position_z 
dd = sqr stdev_initial_velocity 
ee = sqr stdev_initial_attituderoll_pitch
rr = sqr stdev_initial_attitude_yaw

type EkfMatrix = Array 7 (Array 7 SFloat)

pinit :: EkfMatrix

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

step :: VehicleState

step = VehicleState vz vdx vdy vdz phi theta psi where

    shouldPredict = ekfMode == mode_predict && nowMsec >= nextPredictionMsec

    -- XXX need to compute these for real
    tmpq0 = 1
    tmpq1 = 1
    tmpq2 = 1
    tmpq3 = 1
    norm = 1
    isErrorSufficient = ekfMode == mode_finalize && true

    -- Quaternion
    qw = updateQuatValue shouldPredict isErrorSufficient tmpq0 norm 1 _qw
    qx = updateQuatValue shouldPredict isErrorSufficient tmpq1 norm 0 _qx
    qy = updateQuatValue shouldPredict isErrorSufficient tmpq2 norm 0 _qy
    qz = updateQuatValue shouldPredict isErrorSufficient tmpq3 norm 0 _qz

    -- Rotation vector
    rx = updateRotationValue 0 _rx (2*_qx*_qz - 2*_qw*_qy)
    ry = updateRotationValue 0 _ry (2*_qy*_qz + 2*_qw*_qx)
    rz = updateRotationValue 1 _rz (_qw*_qw - _qx*_qx - _qy*_qy + _qz*_qz)

    -- EKF state
    zz = updateEkfValue shouldPredict _zz 0

    dx = updateEkfValue shouldPredict _dx 0

    dy = updateEkfValue shouldPredict _dy 0

    dz = updateEkfValue shouldPredict _dz 0

    e0 = updateEkfValue shouldPredict _e0 0

    e1 = updateEkfValue shouldPredict _e1 0

    e2 = updateEkfValue shouldPredict _e2 0

    vz = 0
    vdx = 0
    vdy = 0
    vdz = 0
    phi = 0
    theta = 0
    psi = 0

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

------------------------------------------------------------------------------

spec = do

  let vstate =  step

  trigger "setState" 
           (ekfMode == mode_get_state) 
           [arg $ vz vstate, 
            arg $ vdx vstate,
            arg $ vdy vstate,
            arg $ vdz vstate,
            arg $ phi vstate,
            arg $ theta vstate,
            arg $ psi vstate]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
