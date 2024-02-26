{--
  LambdaFlight core algorithm for Crazyflie
 
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
import YawAngle
import YawRate

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "finalDemands" Nothing

tmpDemandsStruct :: Stream DemandsStruct
tmpDemandsStruct = extern "tmpDemands" Nothing

resetPids :: SBool
resetPids = extern "resetPids" Nothing

inHoverMode :: SBool
inHoverMode = extern "inHoverMode" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "vehicleState" Nothing

-- Main ----------------------------------------------------------------------

spec = do

  -- Constants 
  let clock_rate = RATE_500_HZ
  let tbase = 36000
  let tscale = 1000
  let tmin = 2000
  let prscale = 1
  let yscale = 1

  let vehicleState = liftState stateStruct

  let demands = liftDemands demandsStruct

  let dt = rateToPeriod clock_rate

  -- let tmpDemands = liftDemands tmpDemandsStruct
  -- let tmpDemands' = pitchRollRatePid resetPids inHoverMode dt vehicleState tmpDemands
  -- trigger "reportHaskell" true [arg $ pitch tmpDemands']
  -- trigger "report" true []

  let pids = [pitchRollAnglePid resetPids inHoverMode dt
             ,pitchRollRatePid resetPids inHoverMode dt
             ,altitudePid inHoverMode dt 
             ,climbRatePid inHoverMode dt
             ,yawAnglePid dt
             ,yawRatePid dt]

  let demands' = foldl (\demand pid -> pid vehicleState demand) demands pids

  let thrust'' = if inHoverMode then ((thrust demands') * tscale + tbase) else tmin

  let motors = quadCFMixer $ Demands thrust''
                                     ((roll demands') * prscale )
                                     ((pitch demands') * prscale )
                                     ((yaw demands') * yscale )

  trigger "setMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
