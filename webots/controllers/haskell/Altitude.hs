{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Altitude where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

runAltitudePid :: ClosedLoopController

runAltitudePid _ state demands = demands'  where

    kp = 2
    ki = 0.5
    dt = 0.01

    integral_limit = 5000

    -- In hover mode, thrust demand comes in as [-1,+1], so
    -- we convert it to a target altitude in meters
    thrust' = rescale (thrust demands) (-1) 1 0.2 2.0

    error = thrust' - (z state)
    
    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    demands' = Demands (kp * error + ki * integ) 
                       (roll demands)
                       (pitch demands)
                       (yaw demands)
          
    integ' = [0] ++ integ
