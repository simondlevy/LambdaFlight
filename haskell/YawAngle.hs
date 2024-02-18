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

yawAnglePid hover dt state demands = demands'

    where 

      kp = 6
      ki = 1
      kd = 0.25
      ilimit = 360
      angle_max = 200

      -- Yaw angle psi is positive nose-left, whereas yaw demand is
      -- positive nose-right.  Hence we negate the yaw demand to
      -- accumulate the angle target.
      target = cap $ target' - angle_max * (yaw demands) * dt

      (yaw', error, integ) =
        pidController kp ki kd dt ilimit target (psi state) cap error' integ'

      -- Return the result negated, so demand will still be nose-right positive
      --yaw' = -(kp * error + ki * integ + kd * deriv) 
      demands' = Demands (thrust demands) (roll demands) (pitch demands) (-yaw')

      target' = [0] ++ target
      integ' = [0] ++ integ
      error' = [0] ++ error
