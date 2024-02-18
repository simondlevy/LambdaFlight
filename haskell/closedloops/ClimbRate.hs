{--
  Climb-rate algorithm for real and simulated flight controllers
 
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

module ClimbRate where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

run dt thrust target base scale minval maxval = thrust'  where

    kp = 25
    ki = 15
    ilimit = 5000

    (thrust', integ) = piController kp ki dt ilimit thrust target integ'

    integ' = [0] ++ integ


climbRatePid :: SFloat -> SFloat -> SFloat -> SFloat -> ClosedLoopController

climbRatePid base scale minval maxval hover dt state demands = demands' where

    thrust' = thrust demands

    -- In hover mode, we scale the thrust so as to keep the vehicle level; 
    -- otherwise, we just scale it by its maximum value
    thrust'' = if hover
               then run dt thrust' (dz state) base scale minval maxval
               else thrust' * maxval

    demands' = Demands thrust'' (roll demands) (pitch demands) (yaw demands)
