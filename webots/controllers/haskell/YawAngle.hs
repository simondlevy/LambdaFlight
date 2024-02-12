{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module YawAngle where

import Language.Copilot
import Copilot.Compile.C99

import Utils

cap :: SFloat -> SFloat

cap angle = angle''

  where angle' = if angle > 180 then angle - 360 else angle

        angle'' = if angle' < (-180) then angle' + 360 else angle'


------------------------------------------------------------------------------

-- Demand is input as desired angle normalized to [-1,+1] and output
-- as degrees per second, both nose-right positive.

runYawAnglePid :: SFloat -> SFloat -> SFloat

runYawAnglePid yawDemand angle = -(kp * error + ki * integ + kd * deriv) 
  -- Return the result negated, so demand will still be nose-right positive

    where 

      kp = 6
      ki = 1
      kd = 0.25
      dt = 0.01
      integral_limit = 360
      demand_angle_max = 200

      -- Yaw angle psi is positive nose-left, whereas yaw demand is
      -- positive nose-right.  Hence we negate the yaw demand to
      -- accumulate the angle target.
      target = cap $ target' - demand_angle_max * yawDemand * dt

      error = cap $ target - angle

      integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

      deriv = (error - error') / dt

      integ' = [0] ++ integ

      target' = [0] ++ target

      error' = [0] ++ error
