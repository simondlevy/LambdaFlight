{--
  LambdaFlight core algorithm: reads open-loop demands and
  state as streams runs PID controllers and motor mixers
 
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

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Copilot where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import Madgwick
import Mixers
import Motors
import Utils

-- Streams from C++ ----------------------------------------------------------

thro_des :: SFloat
thro_des = extern "thro_des" Nothing

roll_des :: SFloat
roll_des = extern "roll_des" Nothing

pitch_des :: SFloat
pitch_des = extern "pitch_des" Nothing

yaw_des :: SFloat
yaw_des = extern "yaw_des" Nothing

roll_IMU :: SFloat
roll_IMU = extern "roll_IMU" Nothing

pitch_IMU :: SFloat
pitch_IMU = extern "pitch_IMU" Nothing

dt :: SFloat
dt = extern "dt" Nothing

gyroX :: SFloat
gyroX = extern "gyroX" Nothing

gyroY :: SFloat
gyroY = extern "gyroY" Nothing

gyroZ :: SFloat
gyroZ = extern "gyroZ" Nothing

accelX :: SFloat
accelX = extern "accelX" Nothing

accelY :: SFloat
accelY = extern "accelY" Nothing

accelZ :: SFloat
accelZ = extern "accelZ" Nothing

-- Tuning constants ---------------------------------------------------------

i_limit = 25 :: SFloat

kp_cyclic = 0.2 :: SFloat  
ki_cyclic = 0.3 :: SFloat
kd_cyclic = 0.05 :: SFloat  

kp_yaw = 0.3 :: SFloat       
ki_yaw = 0.05 :: SFloat      
kd_yaw = 0.00015 :: SFloat

-----------------------------------------------------------------------------

step = motors where

  throttle_is_down = thro_des < 0.06

  -- Roll --------------------------------------------------------

  error_roll = roll_des - roll_IMU

  -- Don't let integrator build if throttle is too low
  integral_roll = if throttle_is_down then 0 else

    -- Avoid integral windup
    constrain (integral_roll' + error_roll * dt) (-i_limit) i_limit

  derivative_roll = gyroX

  -- Scale by .01 to bring within -1 to 1 range
  roll_PID = 0.01 * (kp_cyclic * error_roll + 
        ki_cyclic * integral_roll - 
        kd_cyclic * derivative_roll) 

  -- Pitch -------------------------------------------------------

  error_pitch = pitch_des - pitch_IMU

  -- Don't let integrator build if throttle is too low
  integral_pitch = if throttle_is_down then 0 else

    -- Avoid integral windwup 
    constrain (integral_pitch' + error_pitch * dt) (-i_limit) i_limit

  derivative_pitch = gyroY

  -- Scale by .01 to bring within -1 to 1 range
  pitch_PID = 0.01 * (kp_cyclic * error_pitch + 
         ki_cyclic * integral_pitch - 
         kd_cyclic * derivative_pitch)

  -- Yaw: stablize on rate from gyroZ -------------------------------

  error_yaw = yaw_des - gyroZ

  -- Don't let integrator build if throttle is too low
  integral_yaw = if throttle_is_down then 0 else

    -- Avoid integral windup
    constrain (integral_yaw' + error_yaw * dt) (-i_limit) i_limit

  derivative_yaw = (error_yaw - error_yaw') / dt 

  -- Scale by .01 to bring within -1 to 1 range
  yaw_PID = 0.01 * 
    (kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw) 

  -- Run demands through motor mixer
  motors = quadDFMixer $ Demands thro_des roll_PID pitch_PID yaw_PID

  -- State variables ---------------------------------------------------------

  integral_roll' = [0] ++ integral_roll
  integral_pitch' = [0] ++ integral_pitch
  integral_yaw' = [0] ++ integral_yaw
  error_yaw' = [0] ++ error_yaw

------------------------------------------------------------------------------

spec = do

  let (phi, theta, psi) = madgwick6DOF (gyroX, (-gyroY), (-gyroZ)) 
                                       ((-accelX), accelY, accelZ)
                                       dt

  trigger "setAngles" true [ arg phi, arg theta, arg psi ]

  let motors = step

  trigger "setMotors" true [
      arg $ Motors.qm1 motors, 
      arg $ Motors.qm2 motors, 
      arg $ Motors.qm3 motors, 
      arg $ Motors.qm4 motors] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step" ".") "copilot"