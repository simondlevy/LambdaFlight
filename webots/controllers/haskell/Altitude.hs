{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Altitude where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils


run dt thrust altitude = thrustpid where

    kp = 2
    ki = 0.5
    ilimit = 5000

    -- In hover mode, thrust demand comes in as [-1,+1], so
    -- we convert it to a target altitude in meters
    target = rescale thrust (-1) 1 0.2 2.0

    (thrustpid, integ) = piController kp ki dt ilimit target altitude integ'

    integ' = [0] ++ integ


altitudePid :: ClosedLoopController

altitudePid hover dt state demands = demands'  where

    thrustraw = thrust demands
    
    thrustout = if hover then run dt thrustraw (z state) else thrustraw

    demands' = Demands thrustout
                       (roll demands)
                       (pitch demands)
                       (yaw demands)
