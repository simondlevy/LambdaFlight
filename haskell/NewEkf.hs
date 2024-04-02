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
    ezz :: SFloat
  , edx :: SFloat
  , edy :: SFloat
  , edz :: SFloat
  , e0 :: SFloat
  , e1 :: SFloat
  , e2 :: SFloat
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

data Ekf = Ekf {
    p :: EkfMatrix
  , quat :: Quaternion
  , ekfState :: EkfState
  , gyroSubSampler :: SubSampler
  , accelSubSampler :: SubSampler
}

-----------------------------------------------------------------------------

predict :: SInt32 -> Ekf  -> Ekf
predict lastPredictionMsec ekf = ekf' where

  ekf' = ekf

------------------------------------------------------------------------------

step :: VehicleState

step = vehicleState where

  vehicleState = VehicleState 0 0 0 0 0 0 0

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
