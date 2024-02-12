{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module PitchRollRate where

import Language.Copilot
import Copilot.Compile.C99

import Utils

-------------------------------------------------------------------------------

runRollRatePid :: Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float

runRollRatePid kp ki kd dt integral_limit demand rate =
  kp * error + ki * integ + kd * deriv

  where 

    error = demand - rate

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    deriv = (error - error') / dt

    integ' = [0] ++ integ

    error' = [0] ++ error

-------------------------------------------------------------------------------

runPitchRatePid :: Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float

runPitchRatePid kp ki kd dt integral_limit demand rate =
  kp * error + ki * integ + kd * deriv

  where 

    error = demand - rate

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    deriv = (error - error') / dt

    integ' = [0] ++ integ

    error' = [0] ++ error

------------------------------------------------------------------------------

runPitchRollRatePid :: (Stream Float, Stream Float) -> (Stream Float, Stream Float) ->
  (Stream Float, Stream Float)

runPitchRollRatePid (rollDemand, pitchDemand) (rollRate, pitchRate) =
  (rollDemand', pitchDemand') 

  where kp = 125
        ki = 250
        kd = 1.25
        dt = 0.01
        integral_limit = 33

        rollDemand'  = runRollRatePid  kp ki kd dt integral_limit rollDemand  rollRate
        pitchDemand' = runPitchRatePid kp ki kd dt integral_limit pitchDemand pitchRate
