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


import ClosedLoop
import Demands
import State
import Utils

-------------------------------------------------------------------------------

runRollRatePid kp ki kd reset dt ilimit demand rate = demand'

  where 

    (demand', error, integ) = 
      pidController kp ki kd dt ilimit demand rate id error' integ'

    integ' = [0] ++ (if reset then 0 else integ)
    error' = [0] ++ (if reset then 0 else error)

-------------------------------------------------------------------------------

pitchRatePid kp ki kd reset dt ilimit demand rate = demand'

  where 

    (demand', error, integ) = 
      pidController kp ki kd dt ilimit demand rate id error' integ'

    -- Reset error integral and previous value on reset thrust
    integ' = [0] ++ (if reset then 0 else integ)
    error' = [0] ++ (if reset then 0 else error)

------------------------------------------------------------------------------

pitchRollRatePid :: SBool -> ClosedLoopController

pitchRollRatePid reset hover dt state demands = demands' where

  kp = 125
  ki = 250
  kd = 1.25
  ilimit = 33

  thrust' = thrust demands
  roll'   = roll demands
  pitch'  = pitch demands
  dphi'   = dphi state
  dtheta' = dtheta state

  roll''  = if thrust' == 0
           then 0
           else runRollRatePid kp ki kd reset dt ilimit roll' dphi'

  pitch'' = if thrust' == 0
           then 0
           else pitchRatePid kp ki kd reset dt ilimit pitch' dtheta'

  demands' = Demands thrust' roll'' pitch'' (yaw demands)


