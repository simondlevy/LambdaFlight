{--
  Yaw rate PID control algorithm for real and simulated flight controllers
 
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

module YawRate where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

-- Yaw demand is nose-right positive, whereas yaw angle psi and its first
-- derivative (angular velocity) dspi are nose-right negative.
-- Hence we negate yaw demand, run the PID closedloop on the
-- negated demand and the angular velocity, and negate the result to get the
-- correct yaw demand.

yawRatePid dt state demands = demands' where

    kp = 120
    ki = 16.7
    ilimit = 166.7

    (yaw', integ) = piController kp ki dt ilimit (-(yaw demands)) (dpsi state) integ'

    -- No yaw demand on zero thrust
    yaw'' = if (thrust demands) == 0 then 0 else yaw'

    demands' = Demands (thrust demands) (roll demands) (pitch demands) (-yaw'')

    integ' = [0] ++ integ
