{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module PitchRollRate where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

-------------------------------------------------------------------------------

runRollRatePid :: SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat

runRollRatePid kp ki kd dt integral_limit demand rate =
  kp * error + ki * integ + kd * deriv

  where 

    error = demand - rate

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    deriv = (error - error') / dt

    integ' = [0] ++ integ

    error' = [0] ++ error

-------------------------------------------------------------------------------

pitchRatePid :: SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat

pitchRatePid kp ki kd dt integral_limit demand rate =
  kp * error + ki * integ + kd * deriv

  where 

    error = demand - rate

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    deriv = (error - error') / dt

    integ' = [0] ++ integ

    error' = [0] ++ error

------------------------------------------------------------------------------

pitchRollRatePid :: ClosedLoopController

pitchRollRatePid state demands = demands' where

  kp = 125
  ki = 250
  kd = 1.25
  dt = 0.01
  integral_limit = 33

  roll'  = runRollRatePid  kp ki kd dt integral_limit (roll demands) (dphi state)
  pitch' = pitchRatePid kp ki kd dt integral_limit (pitch demands) (dtheta state)

  demands' = Demands (thrust demands) roll' pitch' (yaw demands)


