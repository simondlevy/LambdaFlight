{--
  Altitude PID control algorithm for real and simulated flight controllers
 
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

module Altitude where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

{-- 

  Demand is input as normalized altitude target in meters and output as 
  climb rate in meters-per-second

--}

altitudePid flying dt state demands = demands'  where

  kp = 2.0
  ki = 0.5
  ilimit = 5000

  thrustraw = thrust demands

  -- In flying mode, thrust demand comes in as [-1,+1], so we convert it to a 
  -- target altitude in meters
  target = rescale thrustraw (-1) 1 0.2 2.0

  error = target - (zz state)

  -- Reset integral when not in flying mode (flying)
  integ = if flying 
          then constrain (integ' + error * dt) (-ilimit) ilimit
          else 0

  integ' = [0] ++ integ

  thrustout = if flying then kp * error + ki * integ else thrustraw

  demands' = Demands thrustout (roll demands) (pitch demands) (yaw demands)
