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

    integral_limit = 5000

    error = thrust - target

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    thrust' = kp * error + ki * integ

    integ' = [0] ++ integ


climbRatePid :: SBool -> SFloat -> SFloat -> SFloat -> SFloat -> ClosedLoopController

climbRatePid inHoverMode base scale minval maxval dt state demands = demands' where

    thrust' = thrust demands

    -- In hover mode, we scale the thrust so as to keep the vehicle level; 
    -- otherwise, we just scale it by its maximum value
    thrust'' = if inHoverMode
               then run dt thrust' (dz state) base scale minval maxval
               else thrust' * maxval

    demands' = Demands thrust'' (roll demands) (pitch demands) (yaw demands)
