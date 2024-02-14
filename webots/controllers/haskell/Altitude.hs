{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Altitude where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils


run :: SFloat -> SFloat -> SFloat

run thrust altitude = thrust'' where

    kp = 2
    ki = 0.5
    dt = 0.01

    integral_limit = 5000

    -- In hover mode, thrust demand comes in as [-1,+1], so
    -- we convert it to a target altitude in meters
    thrust' = rescale thrust (-1) 1 0.2 2.0

    error = thrust' - altitude
    
    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    thrust'' = kp * error + ki * integ
         
    integ' = [0] ++ integ



altitudePid :: SBool -> ClosedLoopController

altitudePid inHoverMode state demands = demands'  where

    thrust' = thrust demands
    
    thrust'' = run thrust' (z state)

    demands' = Demands thrust''
                       (roll demands)
                       (pitch demands)
                       (yaw demands)
