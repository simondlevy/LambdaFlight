{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module YawAngle where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

cap :: Stream Float -> Stream Float

cap angle = angle''

  where angle' = if angle > 180 then angle - 360 else angle

        angle'' = if angle' < (-180) then angle' + 360 else angle'


------------------------------------------------------------------------------

-- Demand is input as desired angle normalized to [-1,+1] and output
-- as degrees per second, both nose-right positive.

runYawAnglePid :: Stream Float -> Stream Float -> Stream Float -> Stream Float

runYawAnglePid angle yawDemand thrustDemand = 
  kp * error + ki * integ + kd * deriv

    where 

      kp = 6
      ki = 1
      kd = 0.25
      dt = 0.01
      integral_limit = 360
      demand_angle_max = 200

      target = cap $ target' - demand_angle_max * yawDemand * dt

      error = cap $ target - angle

      target' = [0] ++ (if thrustDemand == 0 then angle else target)

      integ = 0
      deriv = 0
