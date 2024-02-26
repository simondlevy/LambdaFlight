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

module PitchRollRate where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

runPid kp ki kd ilimit dt error errorErrorPrev errorErrorInteg = output where

    deriv = (error - errorErrorPrev) / dt

    output = kp * error + ki * errorErrorInteg + kd * deriv


{--

  Demands are input as angular velocities in degrees per second and output as
  as arbitrary values to be scaled according to motor characteristics:

  roll:  input roll-right positive => output positive

  pitch: input nose-up positive => output positive

--}

pitchRollRatePid reset hover dt state demands = demands' where

  kp = 125
  ki = 250
  kd = 1.25
  ilimit = 33

  isThrustZero = (thrust demands) == 0

  -------------------------------------------------------------------

  rollError = (roll demands) - (dphi state)

  rollDemand = if isThrustZero then 0
               else runPid kp ki kd ilimit dt rollError rollErrorPrev' rollErrorInteg'

  rollErrorPrev = if reset then 0 
                  else if isThrustZero then rollErrorPrev'
                  else rollError

  rollErrorInteg = if reset  then 0
                   else if isThrustZero then rollErrorInteg'
                   else constrain (rollErrorInteg' + rollError * dt) (-ilimit) ilimit

  rollErrorInteg' = [0] ++ rollErrorInteg

  rollErrorPrev' = [0] ++ rollErrorPrev

  -------------------------------------------------------------------

  pitchError = (pitch demands) - (dtheta state)

  pitchDemand = if isThrustZero then 0
               else runPid kp ki kd ilimit dt pitchError pitchErrorPrev' pitchErrorInteg'

  pitchErrorPrev = if reset then 0 
              else if isThrustZero then pitchErrorPrev'
              else pitchError

  pitchErrorInteg = if reset  then 0
               else if isThrustZero then pitchErrorInteg'
               else constrain (pitchErrorInteg' + pitchError * dt) (-ilimit) ilimit

  pitchErrorInteg' = [0] ++ pitchErrorInteg

  pitchErrorPrev' = [0] ++ pitchErrorPrev


  -------------------------------------------------------------------

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)
