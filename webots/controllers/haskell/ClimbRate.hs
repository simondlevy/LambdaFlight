{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module ClimbRate where

import Language.Copilot
import Copilot.Compile.C99

import Utils

runClimbRatePid :: SFloat -> SFloat -> SFloat
runClimbRatePid climbRate dz = kp * error + ki * integ

  where

    kp = 25
    ki = 15
    dt = 0.01

    integral_limit = 5000

    error = climbRate - dz

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    integ' = [0] ++ integ

