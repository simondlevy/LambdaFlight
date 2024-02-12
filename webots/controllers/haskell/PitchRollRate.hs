{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module PitchRollRate where

import Language.Copilot
import Copilot.Compile.C99

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

runPitchRatePid :: SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat

runPitchRatePid kp ki kd dt integral_limit demand rate =
  kp * error + ki * integ + kd * deriv

  where 

    error = demand - rate

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    deriv = (error - error') / dt

    integ' = [0] ++ integ

    error' = [0] ++ error

------------------------------------------------------------------------------

runPitchRollRatePid :: (SFloat, SFloat) -> (SFloat, SFloat) ->
  (SFloat, SFloat)

runPitchRollRatePid (rollDemand, pitchDemand) (rollRate, pitchRate) =
  (rollDemand', pitchDemand') 

  where kp = 125
        ki = 250
        kd = 1.25
        dt = 0.01
        integral_limit = 33

        rollDemand'  = runRollRatePid  kp ki kd dt integral_limit rollDemand  rollRate
        pitchDemand' = runPitchRatePid kp ki kd dt integral_limit pitchDemand pitchRate