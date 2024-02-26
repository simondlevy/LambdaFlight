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

module Core where

import Language.Copilot
import Copilot.Compile.C99

import Clock
import Demands
import Mixers
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

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "openLoopDemands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "vehicleState" Nothing

inHoverMode :: SBool
inHoverMode = extern "inHoverMode" Nothing

resetPids :: SBool
resetPids = extern "resetPids" Nothing

step clock_rate tbase tscale tmin prscale yscale = motors where

  vehicleState = liftState stateStruct

  openLoopDemands = liftDemands demandsStruct

  dt = rateToPeriod clock_rate

  pids = [positionPid resetPids inHoverMode dt,
          pitchRollAnglePid resetPids inHoverMode dt,
          pitchRollRatePid resetPids inHoverMode dt,
          altitudePid inHoverMode dt,
          climbRatePid inHoverMode dt,
          yawAnglePid dt,
          yawRatePid dt]

  demands' = foldl (\demand pid -> pid vehicleState demand) openLoopDemands pids

  thrust'' = if inHoverMode then ((thrust demands') * tscale + tbase) else tmin

  motors = quadCFMixer $ Demands thrust''
                                 ((roll demands') * prscale)
                                 ((pitch demands') * prscale)
                                 ((yaw demands') * yscale)
