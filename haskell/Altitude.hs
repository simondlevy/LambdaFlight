{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Altitude where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils


altitudePid :: ClosedLoopController

altitudePid hover dt state demands = demands'  where

    kp = 2
    ki = 0.5
    ilimit = 5000

    thrustraw = thrust demands

    -- In hover mode, thrust demand comes in as [-1,+1], so
    -- we convert it to a target altitude in meters
    target = rescale thrustraw (-1) 1 0.2 2.0

    (thrustpid, integ) = piController kp ki dt ilimit target (z state) integ'

    integ' = [0] ++ integ
    
    thrustout = if hover then thrustpid else thrustraw

    demands' = Demands thrustout
                       (roll demands)
                       (pitch demands)
                       (yaw demands)
