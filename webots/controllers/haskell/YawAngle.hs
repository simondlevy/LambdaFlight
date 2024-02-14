{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module YawAngle where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

cap angle = angle''

  where angle' = if angle > 180 then angle - 360 else angle

        angle'' = if angle' < (-180) then angle' + 360 else angle'

------------------------------------------------------------------------------

-- Demand is input as desired angle normalized to [-1,+1] and output
-- as degrees per second, both nose-right positive.

yawAnglePid :: ClosedLoopController

yawAnglePid dt state demands = demands'

    where 

      kp = 6
      ki = 1
      kd = 0.25
      integral_limit = 360
      demand_angle_max = 200

      yawDemand = (yaw demands)
      angle = (psi state)

      -- Yaw angle psi is positive nose-left, whereas yaw demand is
      -- positive nose-right.  Hence we negate the yaw demand to
      -- accumulate the angle target.
      target = cap $ target' - demand_angle_max * yawDemand * dt

      error = cap $ target - angle

      integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

      deriv = (error - error') / dt

      -- Return the result negated, so demand will still be nose-right positive
      yaw' = -(kp * error + ki * integ + kd * deriv) 

      demands' = Demands (thrust demands) (roll demands) (pitch demands) yaw'

      integ' = [0] ++ integ
      target' = [0] ++ target
      error' = [0] ++ error
