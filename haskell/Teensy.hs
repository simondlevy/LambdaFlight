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

module Teensy where

import Language.Copilot hiding(atan2, (!!))
import Copilot.Compile.C99

import Demands
import Mixers
import Motors
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

dt :: SFloat
dt = extern "stream_dt" Nothing

qw :: SFloat
qw = extern "stream_quat_w" Nothing

qx :: SFloat
qx = extern "stream_quat_x" Nothing

qy :: SFloat
qy = extern "stream_quat_y" Nothing

qz :: SFloat
qz = extern "stream_quat_z" Nothing

gyro_x :: SFloat
gyro_x = extern "stream_gyro_x" Nothing

gyro_y :: SFloat
gyro_y = extern "stream_gyro_y" Nothing

gyro_z :: SFloat
gyro_z = extern "stream_gyro_z" Nothing

accel_x :: SFloat
accel_x = extern "stream_accel_x" Nothing

accel_y :: SFloat
accel_y = extern "stream_accel_y" Nothing

accel_z :: SFloat
accel_z = extern "stream_accel_z" Nothing

rangefinder_distance :: SFloat
rangefinder_distance = extern "stream_rangefinder_distance" Nothing

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

max_roll = 30 :: SFloat    
max_pitch = 30 :: SFloat   
max_yaw = 160 :: SFloat    

-- State estimation ---------------------------------------------------------


compute_dz = (dz_rangefinder, dz_accel) where

  accel_z_bias = 0.9375 --0.953

  z_rangefinder = rangefinder_distance / 1000 -- mm => m

  dz_rangefinder = (z_rangefinder - z_rangefinder') / dt

  dz_accel = (accel_z - accel_z_bias) * 9.81 * dt + dz_accel'

  z_rangefinder' = [0] ++ z_rangefinder
  dz_accel' = [0] ++ dz_accel

-----------------------------------------------------------------------------

step = (phi, theta, psi, z, dx, dy, dz, m1, m2, m3, m4) where

  -- Get Euler angles from USFS hardware quaternion --------------------------

  phi = (atan2 (qw*qx + qy*qz) (0.5 - qx*qx - qy*qy)) * 180 / pi

  theta = ((-asin (constrain ((-2.0) * (qx*qz - qw*qy)) (-0.999999) 0.999999))
           * 180 / pi)

  psi = (-atan2 (qx*qy + qw*qz) (0.5 - qy*qy - qz*qz)) * 180 / pi

  -- Get X/Y velocity --------------------------------------------------------

  z = 0 :: SFloat
  dx = 0 :: SFloat
  dy = 0 :: SFloat
  dz = 0 :: SFloat

  -- Get open-loop demands --------------------------------------------------

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

  demand_throttle = constrain ((c1 - 1000) / 1000) 0 1
  demand_roll     = (constrain ((c2 - 1500 ) / 500) (-1) 1) * max_roll
  demand_pitch    = (constrain ((c3 - 1500) / 500) (-1) 1) * max_pitch
  demand_yaw      = (constrain (-(c4 - 1500) / 500) (-1) 1) * max_yaw

  -- Roll PID --------------------------------------------------------

  error_roll = demand_roll - phi

  -- Don't let integrator build if throttle is too low
  integral_roll = if throttle_is_down then 0 else

    -- Avoid integral windup
    constrain (integral_roll' + error_roll * dt) (-i_limit) i_limit

  derivative_roll = gyro_x

  -- Scale by .01 to bring within -1 to 1 range
  roll_pid = 0.01 * (kp_cyclic * error_roll + 
        ki_cyclic * integral_roll - 
        kd_cyclic * derivative_roll) 

  -- Pitch PID -------------------------------------------------------

  error_pitch = demand_pitch - theta

  -- Don't let integrator build if throttle is too low
  integral_pitch = if throttle_is_down then 0 else

    -- Avoid integral windwup 
    constrain (integral_pitch' + error_pitch * dt) (-i_limit) i_limit

  derivative_pitch = gyro_y

  -- Scale by .01 to bring within -1 to 1 range
  pitch_pid = 0.01 * (kp_cyclic * error_pitch + 
         ki_cyclic * integral_pitch - 
         kd_cyclic * derivative_pitch)

  -- Yaw PID : stablize on rate from gyroZ -------------------------------

  error_yaw = demand_yaw - gyro_z

  -- Don't let integrator build if throttle is too low
  integral_yaw = if throttle_is_down then 0 else

    -- Avoid integral windup
    constrain (integral_yaw' + error_yaw * dt) (-i_limit) i_limit

  derivative_yaw = (error_yaw - error_yaw') / dt 

  -- Scale by .01 to bring within -1 to 1 range
  yaw_pid = 0.01 * 
    (kp_yaw * error_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw) 

  -- Run demands through motor mixer
  motors = quadDFMixer $ Demands demand_throttle roll_pid pitch_pid yaw_pid

  -- Convert motors to PWM interval, with safety check for arming

  safeMotor m = if armed then constrain ((m motors) * 125 + 125) 125 250 else 120

  m1 = safeMotor Motors.qm1 
  m2 = safeMotor Motors.qm2 
  m3 = safeMotor Motors.qm3 
  m4 = safeMotor Motors.qm4 

  -- State variables ---------------------------------------------------------

  integral_roll' = [0] ++ integral_roll
  integral_pitch' = [0] ++ integral_pitch
  integral_yaw' = [0] ++ integral_yaw
  error_yaw' = [0] ++ error_yaw
  armed' = [False] ++ armed

------------------------------------------------------------------------------

spec = do

  let (phi, theta, psi, z, dx, dy, dz, m1, m2, m3, m4) = step

  let (dz_rangefinder, dz_accel) = compute_dz

  trigger "setDz" true [arg dz_rangefinder, arg dz_accel]

  trigger "setState" true [arg phi, arg theta, arg psi, arg z, arg dx, arg dy, arg dz]

  trigger "setMotors" true [arg m1, arg m2, arg m3, arg m4] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step" ".") "copilot"
