{--
  LambdaFlight core algorithm: reads open-loop demands and
  state as streams; runs PID controllers and motor mixers
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Core where

import Language.Copilot
import Copilot.Compile.C99

import Demands
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

throttle_is_down :: SBool
throttle_is_down = extern "throttle_is_down" Nothing

gyroX :: SFloat
gyroX = extern "gyroX" Nothing

gyroY :: SFloat
gyroY = extern "gyroY" Nothing

gyroZ :: SFloat
gyroZ = extern "gyroZ" Nothing

-----------------------------------------------------------------------------

step = motors where

  motors = quadCFMixer $ Demands 0 0 0 0

------------------------------------------------------------------------------

spec = do

  let motors = step

  trigger "setMotors" true [
      arg $ Motors.qm1 motors, 
      arg $ Motors.qm2 motors, 
      arg $ Motors.qm3 motors, 
      arg $ Motors.qm4 motors] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
