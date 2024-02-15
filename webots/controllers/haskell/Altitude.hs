{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Altitude where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils


run dt thrust altitude = thrust'' where

    kp = 2
    ki = 0.5
    ilimit = 5000

    -- In hover mode, thrust demand comes in as [-1,+1], so
    -- we convert it to a target altitude in meters
    thrust' = rescale thrust (-1) 1 0.2 2.0

    (thrust'', integ) = piController kp ki dt ilimit thrust' altitude integ'

    integ' = [0] ++ integ


altitudePid :: SBool -> ClosedLoopController

altitudePid inHoverMode dt state demands = demands'  where

    thrust' = thrust demands
    
    thrust'' = if inHoverMode then run dt thrust' (z state) else thrust'

    demands' = Demands thrust''
                       (roll demands)
                       (pitch demands)
                       (yaw demands)
