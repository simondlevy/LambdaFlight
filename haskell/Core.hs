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

import Clock
import Motors

import Constants

import Clock
import Demands
import Mixers
import Motors
import Sensors
import State
import Utils

import Constants

-- PID controllers
import Altitude
import ClimbRate
import PitchRollAngle
import PitchRollRate
import Position
import YawAngle
import YawRate

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "stream_openLoopDemands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "stream_vehicleState" Nothing

inFlyingMode :: SBool
inFlyingMode = extern "stream_inFlyingMode" Nothing

resetPids :: SBool
resetPids = extern "stream_resetPids" Nothing

step = motors where

  vehicleState = liftState stateStruct

  openLoopDemands = liftDemands demandsStruct

  dt = rateToPeriod clock_rate

  pids = [positionPid resetPids inFlyingMode dt,
          pitchRollAnglePid resetPids dt,
          pitchRollRatePid resetPids dt,
          altitudePid inFlyingMode dt,
          climbRatePid inFlyingMode dt,
          yawAnglePid dt,
          yawRatePid dt]

  demands' = foldl (\demand pid -> pid vehicleState demand) openLoopDemands pids

  thrust'' = if inFlyingMode then ((thrust demands') * tscale + tbase) else tmin

  motors = quadCFMixer $ Demands thrust''
                                 ((roll demands') * prscale)
                                 ((pitch demands') * prscale)
                                 ((yaw demands') * yscale)

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
