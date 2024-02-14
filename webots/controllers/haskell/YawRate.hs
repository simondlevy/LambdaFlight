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

yawRatePid :: ClosedLoopController

yawRatePid dt state demands = demands' where

    kp = 120
    ki = 16.7
    integral_limit = 166.7

    desired = yaw demands
    measured = dpsi state

    error = (-desired) - measured

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    yaw' = -(kp * error + ki * integ)

    demands' = Demands (thrust demands) (roll demands) (pitch demands) yaw'

    integ' = [0] ++ integ
