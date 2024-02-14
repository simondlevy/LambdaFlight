{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module ClimbRate where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

run :: SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat

run thrust target base scale minval maxval = thrust'  where

    kp = 25
    ki = 15
    dt = 0.01

    integral_limit = 5000

    error = thrust - target

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    thrust' = kp * error + ki * integ

    integ' = [0] ++ integ



climbRatePid :: SBool -> SFloat -> SFloat -> SFloat -> SFloat -> ClosedLoopController

climbRatePid inHoverMode base scale minval maxval state demands = demands' where

    thrust' = thrust demands

    thrust'' = if inHoverMode
               then run thrust' (dz state) base scale minval maxval
               else thrust' * maxval

    demands' = Demands thrust'' (roll demands) (pitch demands) (yaw demands)
