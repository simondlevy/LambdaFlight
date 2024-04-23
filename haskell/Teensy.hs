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
import MinEkf
import Mixers
import Motors
import State
import Utils

-- Streams from C++ ----------------------------------------------------------

channel1_raw :: SFloat
channel1_raw = extern "stream_channel1_raw" Nothing

channel2_raw :: SFloat
channel2_raw = extern "stream_channel2_raw" Nothing

channel3_raw :: SFloat
channel3_raw = extern "stream_channel3_raw" Nothing

channel4_raw :: SFloat
channel4_raw = extern "stream_channel4_raw" Nothing

channel5_raw :: SFloat
channel5_raw = extern "stream_channel5_raw" Nothing

radio_failsafe :: SBool
radio_failsafe = extern "stream_radio_failsafe" Nothing

statePhi :: SFloat
statePhi = extern "stream_state_phi" Nothing

stateTheta :: SFloat
stateTheta = extern "stream_state_theta" Nothing

dt :: SFloat
dt = extern "stream_dt" Nothing

-- PID tuning constants -----------------------------------------------------

i_limit = 25 :: SFloat

kp_cyclic = 0.2 :: SFloat  
ki_cyclic = 0.3 :: SFloat
kd_cyclic = 0.05 :: SFloat  

kp_yaw = 0.3 :: SFloat       
ki_yaw = 0.05 :: SFloat      
kd_yaw = 0.00015 :: SFloat

-- SBUS scaling -------------------------------------------------------------

sbus_scale = 0.615  :: SFloat
sbus_bias  = 895.0 :: SFloat

pwm_min = 800 :: SFloat
pwm_max = 2200 :: SFloat

maxRoll = 30 :: SFloat    
maxPitch = 30 :: SFloat   
maxYaw = 160 :: SFloat    

-- IMU LP filter parameters
b_gyro = 0.1 :: SFloat

-----------------------------------------------------------------------------

getGyro :: (SFloat, SFloat, SFloat)

getGyro = (gyroX, gyroY, gyroZ) where

  lpf = \v v' b -> let s = v in (1 - b) * v' + b * s

  glpf = \g g' -> lpf g g' b_gyro

  gyroX = glpf stream_gyro_x gyroX'
  gyroY = glpf stream_gyro_y gyroY'
  gyroZ = glpf stream_gyro_z gyroZ'

  gyroX' = [0] ++ gyroX
  gyroY' = [0] ++ gyroY
  gyroZ' = [0] ++ gyroZ

-----------------------------------------------------------------------------

step gyroX gyroY gyroZ = motors' where

  -- Open-loop demands ----------------------------------------------

  scaleup = \c -> sbus_scale * c + sbus_bias

  c1 = scaleup channel1_raw
  c2 = scaleup channel2_raw
  c3 = scaleup channel3_raw
  c4 = scaleup channel4_raw
  c5 = scaleup channel5_raw

  throttle_is_down = c1 < 1060

  armed = if radio_failsafe then false
          else if c5 < 1500 then false 
          else if c1 < 1050 then true 
          else armed'

  demandThrottle = constrain ((c1 - 1000) / 1000) 0 1
  demandRoll     = (constrain ((c2 - 1500 ) / 500) (-1) 1) * maxRoll
  demandPitch    = (constrain ((c3 - 1500) / 500) (-1) 1) * maxPitch
  demandYaw      = (constrain (-(c4 - 1500) / 500) (-1) 1) * maxYaw

  -- Roll PID --------------------------------------------------------

  error_roll = demandRoll - statePhi

  -- Don't let integrator build if throttle is too low
  integral_roll = if throttle_is_down then 0 else

    -- Avoid integral windup
    constrain (integral_roll' + error_roll * dt) (-i_limit) i_limit

  derivative_roll = gyroX

  -- Scale by .01 to bring within -1 to 1 range
  roll_PID = 0.01 * (kp_cyclic * error_roll + 
        ki_cyclic * integral_roll - 
        kd_cyclic * derivative_roll) 

  -- Pitch PID -------------------------------------------------------

  error_pitch = demandPitch - stateTheta

  -- Don't let integrator build if throttle is too low
  integral_pitch = if throttle_is_down then 0 else

    -- Avoid integral windwup 
    constrain (integral_pitch' + error_pitch * dt) (-i_limit) i_limit

  derivative_pitch = gyroY

  -- Scale by .01 to bring within -1 to 1 range
  pitch_PID = 0.01 * (kp_cyclic * error_pitch + 
         ki_cyclic * integral_pitch - 
         kd_cyclic * derivative_pitch)

  -- Yaw PID : stablize on rate from gyroZ -------------------------------

  error_yaw = demandYaw - gyroZ

  -- Don't let integrator build if throttle is too low
  integral_yaw = if throttle_is_down then 0 else

    -- Avoid integral windup
    constrain (integral_yaw' + error_yaw * dt) (-i_limit) i_limit

  derivative_yaw = (error_yaw - error_yaw') / dt 

  -- Scale by .01 to bring within -1 to 1 range
  yaw_PID = 0.01 * 
    (kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw) 

  -- Run demands through motor mixer
  motors = quadDFMixer $ Demands demandThrottle roll_PID pitch_PID yaw_PID

  -- Convert motors to PWM interval, with safety check for arming

  safeMotor m = if armed then constrain ((m motors) * 125 + 125) 125 250 else 120

  m1_pwm = safeMotor Motors.qm1 
  m2_pwm = safeMotor Motors.qm2 
  m3_pwm = safeMotor Motors.qm3 
  m4_pwm = safeMotor Motors.qm4 

  motors' = (m1_pwm, m2_pwm, m3_pwm, m4_pwm, c1)

  -- State variables ---------------------------------------------------------

  integral_roll' = [0] ++ integral_roll
  integral_pitch' = [0] ++ integral_pitch
  integral_yaw' = [0] ++ integral_yaw
  error_yaw' = [0] ++ error_yaw
  armed' = [False] ++ armed

------------------------------------------------------------------------------

spec = do

  let (gyroX, gyroY, gyroZ) = getGyro

  let (m1_pwm, m2_pwm, m3_pwm, m4_pwm, c1) = step gyroX gyroY gyroZ
  
  let vehicleState = ekfStep

  trigger "setVehicleState" true [arg $ phi vehicleState]

  trigger "setMotors" true [arg m1_pwm, arg m2_pwm, arg m3_pwm, arg m4_pwm] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step" ".") "copilot"
