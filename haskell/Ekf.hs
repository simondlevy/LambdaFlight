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
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds        #-}
{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot hiding(atan2)
import Copilot.Compile.C99

import Utils

-- Initial variances, uncertain of position, but know we're stationary and
-- roughly flat
stdev_initial_position_z          = 1.0  :: Float
stdev_initial_velocity            = 0.01 :: Float
stdev_initial_attitude_roll_pitch = 0.01 :: Float
stdev_initial_attitude_yaw        = 0.01 :: Float

------------------------------------------------------------------------------

type EkfMode = SInt8

ekfMode :: EkfMode
ekfMode = extern "stream_ekfMode" Nothing

nowMsec :: SInt32
nowMsec = extern "stream_nowMsec" Nothing

mode_init              = 0 :: EkfMode
mode_predict           = 1 :: EkfMode
mode_finalize          = 2 :: EkfMode
mode_get_state         = 3 :: EkfMode
mode_update_with_gyro  = 4 :: EkfMode
mode_update_with_accel = 5 :: EkfMode
mode_update_with_range = 6 :: EkfMode
mode_update_with_flow  = 7 :: EkfMode

------------------------------------------------------------------------------

sqr :: Float -> Float
sqr x = x * x

z = sqr stdev_initial_position_z 
d = sqr stdev_initial_velocity 
e = sqr stdev_initial_attitude_roll_pitch
y = sqr stdev_initial_attitude_yaw

type EkfArray = Array 7 (Array 7 Float)

raw_pinit :: EkfArray

raw_pinit =  array [--  z   dx  dy  dz  e0  e1  e2
                 array [z,  0,  0,  0,  0,  0,  0], -- z
                 array [0,  d,  0,  0,  0,  0,  0], -- dx
                 array [0,  0,  d,  0,  0,  0,  0], -- dy
                 array [0,  0,  0,  d,  0,  0,  0], -- dz
                 array [0,  0,  0,  0,  e,  0,  0], -- e0
                 array [0,  0,  0,  0,  0,  e,  0], -- e1
                 array [0,  0,  0,  0,  0,  0,  y]  -- e2
             ] 

type SEkfArray = Stream EkfArray

pinit :: SEkfArray

pinit = [ raw_pinit ] ++ pinit

init :: (SEkfArray, (SFloat, SFloat, SFloat, SFloat))

init = (pinit, (1, 0, 0, 0))

------------------------------------------------------------------------------

step :: (SFloat, SFloat, SFloat, SFloat, SFloat, SFloat) 

step = (dx, dy, dz, phi, theta, psi) where

   init = ekfMode == mode_init

   gyroSubsamplerX = (if init then 0 else gyroSubsamplerX') :: SFloat
   gyroSubsamplerY = (if init then 0 else gyroSubsamplerY') :: SFloat
   gyroSubsamplerZ = (if init then 0 else gyroSubsamplerZ') :: SFloat

   accelSubsamplerX = (if init then 0 else accelSubsamplerX') :: SFloat
   accelSubsamplerY = (if init then 0 else accelSubsamplerY') :: SFloat
   accelSubsamplerZ = (if init then 0 else accelSubsamplerZ') :: SFloat

   pmat = if init then pinit else pmat'

   dx = if init then 0 else dx'

   dy = if init then 0 else dy'

   dz = if init then 0 else r20 * dx' + r21 * dy' + r22 * dz'

   z = (if init then 0 else z') :: SFloat

   qw = if init then 1 else qw'
   qx = if init then 0 else qx'
   qy = if init then 0 else qy'
   qz = if init then 0 else qz'

   -- Set the initial rotation matrix to the identity. This only affects  the
   -- first prediction step, since in the finalization, after shifting 
   -- attitude errors into the attitude state, the rotation matrix is updated.
   r20 = if init then 0 else r20'
   r21 = if init then 0 else r21'
   r22 = if init then 1 else r22'

   isUpdated = if init then false else isUpdated'

   lastPredictionMsec = (if init then nowMsec else lastPredictionMsec') :: SInt32

   lastProcessNoiseUpdateMsec = 
     (if init then nowMsec else lastProcessNoiseUpdateMsec') :: SInt32

   phi = rad2deg $ atan2 (2 * (qy*qz + qw*qx)) (qw*qw - qx*qx - qy*qy + qz*qz)

   -- Negate for ENU
   theta = -(rad2deg $ asin ((-2) * (qx*qz - qw*qy)))

   psi = rad2deg $ atan2 (2 * (qx*qy + qw*qz)) (qw*qw + qx*qx - qy*qy - qz*qz)

   pmat' = [raw_pinit] ++ pmat

   -- EKF state
   dx' = [0] ++ dx
   dy' = [0] ++ dy
   dz' = [0] ++ dz
   z' = [0] ++ z

   -- Quaternion
   qw' = [0] ++ qw
   qx' = [0] ++ qx
   qy' = [0] ++ qy
   qz' = [0] ++ qz

   -- Rotation vector
   r20' = [0] ++ r20
   r21' = [0] ++ r21
   r22' = [1] ++ r22

   -- Gyro subsampler
   gyroSubsamplerX' = [0] ++ gyroSubsamplerX 
   gyroSubsamplerY' = [0] ++ gyroSubsamplerY 
   gyroSubsamplerZ' = [0] ++ gyroSubsamplerZ 

   -- Accel subsampler
   accelSubsamplerX' = [0] ++ accelSubsamplerX 
   accelSubsamplerY' = [0] ++ accelSubsamplerY 
   accelSubsamplerZ' = [0] ++ accelSubsamplerZ 

   isUpdated' = [False] ++ isUpdated
   lastPredictionMsec' = [0] ++ lastPredictionMsec
   lastProcessNoiseUpdateMsec' = [0] ++ lastProcessNoiseUpdateMsec

------------------------------------------------------------------------------

spec = do

  let (dx, dy, dz, phi, theta, psi) = step

  trigger "setState" 
           (ekfMode == mode_get_state) 
           [arg dx, arg dy, arg dz, arg phi, arg theta, arg psi]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
