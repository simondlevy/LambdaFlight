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

module Crazyflie where

import Language.Copilot
import Copilot.Compile.C99

import Clock
import Constants
import Demands
import Mixers
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

step = (motors, openLoopDemands) where

  vehicleState = liftState stateStruct

  openLoopDemands = liftDemands demandsStruct

  dt = rateToPeriod clock_rate

  pids = [positionPid resetPids dt,
          pitchRollAnglePid resetPids dt,
          pitchRollRatePid resetPids dt,
          altitudePid inFlyingMode dt,
          climbRatePid inFlyingMode dt,
          yawAnglePid dt,
          yawRatePid dt]

  demands' = foldl (\demand pid -> pid vehicleState demand) openLoopDemands pids

  thrust'' = if inFlyingMode then ((thrust demands') * tscale + tbase) else tmin

  motors = quadXMixer $ Demands thrust''
                                ((roll demands') * prscale)
                                ((pitch demands') * prscale)
                                ((yaw demands') * yscale)

------------------------------------------------------------------------------
 
spec = do

    let (motors, demands) = step

    let (me_ne, m_se, m_sw, m_nw) = motors

    trigger "setMotors" true [arg $ me_ne, arg $ m_se, arg $ m_sw, arg $ m_nw] 

    trigger "debugDemands" true [arg $ roll demands, arg $ pitch demands]

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
