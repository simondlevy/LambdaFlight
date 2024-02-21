{--
  Miniflie algorithm for real and simulated flight controllers
 
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
import Scaling
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

-- Constants that will be different for sim vs. actual
import Constants

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "demands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "state" Nothing

hover :: SBool
hover = extern "hover" Nothing

-- Main ----------------------------------------------------------------------

spec = do

  let state = liftState stateStruct

  let demands = liftDemands demandsStruct

  let dt = rateToPeriod clock_rate

  let pids = [altitudePid hover dt,
              climbRatePid 
                 (thrust_base constants)
                 (thrust_scale constants)
                 (thrust_min constants)
                 (thrust_max constants)
                 hover dt,
              positionPid (pitch_roll_angle_max constants) hover dt,
              pitchRollAnglePid hover dt, 
              pitchRollRatePid hover dt, 
              yawAnglePid hover dt, 
              yawRatePid hover dt]

  let demands' = foldl (\d f -> f state d) demands pids

  let motors = quadCFMixer $ Demands (thrust demands') 
                                     ((roll demands') * (pitch_roll_scale constants))
                                     ((pitch demands') * (pitch_roll_scale constants))
                                     ((yaw demands') * (yaw_scale constants))

  trigger "report" true [arg $ thrust demands']

  trigger "setMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
