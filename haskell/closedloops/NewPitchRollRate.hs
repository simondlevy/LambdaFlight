{--
  Pitch/roll angular rate PID-control algorithm for real and simulated flight
  controllers
 
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

module NewPitchRollRate where

import Language.Copilot
import Copilot.Compile.C99


import Demands
import State
import Utils

runPid kp ki kd ilimit dt error errorPrev errorInteg = output where

    deriv = (error - errorPrev) / dt

    output = kp * error + ki * errorInteg + kd * deriv


pitchRollRatePid reset hover dt state demands = demands' where

  kp = 125
  ki = 250
  kd = 1.25
  ilimit = 33

  isThrustZero = (thrust demands) == 0

  -------------------------------------------------------------------

  rollError = (roll demands) - (dphi state)

  rollDemand = if isThrustZero then 0
               else runPid kp ki kd ilimit dt rollError rollPrev rollInteg

  rollPrev = if reset then 0 
             else if isThrustZero then rollPrev'
             else rollError

  rollInteg = if reset  then 0
              else if isThrustZero then rollInteg'
              else constrain (rollInteg' + rollError * dt) (-ilimit) ilimit

  rollInteg' = [0] ++ rollInteg

  rollPrev' = [0] ++ rollPrev

  -------------------------------------------------------------------

  pitchError = (pitch demands) - (dtheta state)

  pitchDemand = if isThrustZero then 0
               else runPid kp ki kd ilimit dt pitchError pitchPrev pitchInteg

  pitchPrev = if reset then 0 
              else if isThrustZero then pitchPrev'
              else pitchError

  pitchInteg = if reset  then 0
               else if isThrustZero then pitchInteg'
               else constrain (pitchInteg' + pitchError * dt) (-ilimit) ilimit

  pitchInteg' = [0] ++ pitchInteg

  pitchPrev' = [0] ++ pitchPrev


  -------------------------------------------------------------------

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)
