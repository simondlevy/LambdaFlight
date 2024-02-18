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

-------------------------------------------------------------------------------

runRollAnglePid kp ki dt ilimit demand angle = demand'

  where 

    (demand', integ) = piController kp ki dt ilimit demand angle integ'

    integ' = [0] ++ integ

-------------------------------------------------------------------------------


pitchAnglePid kp ki dt ilimit demand angle = demand'

  where 

    (demand', integ) = piController kp ki dt ilimit demand angle integ'

    integ' = [0] ++ integ

------------------------------------------------------------------------------

pitchRollAnglePid :: ClosedLoopController

pitchRollAnglePid hover dt state demands = demands'

  where kp = 6
        ki = 3
        ilimit = 20

        roll'  = runRollAnglePid  kp ki dt ilimit (roll demands)  (phi state)
        pitch' = pitchAnglePid kp ki dt ilimit (pitch demands) (theta state)

        demands' = Demands (thrust demands) roll' pitch' (yaw demands)
