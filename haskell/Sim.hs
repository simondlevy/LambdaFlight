{--
  LambdaFlight core algorithm for real and simulated flight controllers
 
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

module Main where

import Language.Copilot
import Copilot.Compile.C99

import Clock
import Demands
import Mixers
import Motors
import State
import Utils

-- PID controllers
import Altitude
import ClimbRate
import PitchRollAngle
import PitchRollRate
import Position
import YawAngle
import YawRate

-- Constants -----------------------------------------------------------------

clock_rate = RATE_100_HZ

thrust_base = 48   
thrust_scale = 0.25
thrust_min =   0   
thrust_max =   60   
pitch_roll_scale = 1e-4 
yaw_scale = 4e-5

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "openLoopDemands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "vehicleState" Nothing

inHoverMode :: SBool
inHoverMode = extern "inHoverMode" Nothing

resetPids :: SBool
resetPids = extern "resetPids" Nothing


-- Main ----------------------------------------------------------------------

spec = do

  let vehicleState = liftState stateStruct

  let openLoopDemands = liftDemands demandsStruct

  let dt = rateToPeriod clock_rate

  let pids = [altitudePid inHoverMode dt,
              climbRatePid inHoverMode dt,
              positionPid resetPids inHoverMode dt,
              pitchRollAnglePid resetPids inHoverMode dt, 
              pitchRollRatePid resetPids inHoverMode dt, 
              yawAnglePid dt, 
              yawRatePid dt]

  let demands = foldl (\demand pid -> pid vehicleState demand) openLoopDemands pids

  let thrust' = if inHoverMode 
                 then constrain ((thrust demands) * thrust_scale + thrust_base) 
                      thrust_min 
                      thrust_max
                 else (thrust openLoopDemands) * thrust_max

  let motors = quadCFMixer $ Demands thrust'
                                     ((roll demands) * pitch_roll_scale)
                                     ((pitch demands) * pitch_roll_scale)
                                     ((yaw demands) * yaw_scale)

  trigger "setMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
