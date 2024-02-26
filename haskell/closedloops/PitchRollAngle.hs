{--
  Pitch/roll angle PID-control algorithm for real and simulated flight
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

module PitchRollAngle where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

runPid kp ki ilimit dt error errorErrorInteg = output where

    output = kp * error + ki * errorErrorInteg


{--

  Demand is input as angles in degrees and output as angular velocities in
  degrees per second:

  roll: right-down positive

  pitch: nose-up positive

--}

pitchRollAnglePid reset hover dt state demands = demands' where

  kp = 6
  ki = 3
  ilimit = 20

  -------------------------------------------------------------------

  rollError = (roll demands) - (phi state)

  rollDemand = runPid kp ki ilimit dt rollError rollErrorInteg'

  rollErrorInteg = if reset  then 0
                   else constrain (rollErrorInteg' + rollError * dt) (-ilimit) ilimit

  rollErrorInteg' = [0] ++ rollErrorInteg

  -------------------------------------------------------------------

  pitchError = (pitch demands) - (theta state)

  pitchDemand = runPid kp ki ilimit dt pitchError pitchErrorInteg'

  pitchErrorInteg = if reset  then 0
                    else constrain (pitchErrorInteg' + pitchError * dt) (-ilimit) ilimit

  pitchErrorInteg' = [0] ++ pitchErrorInteg

  -------------------------------------------------------------------

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)

