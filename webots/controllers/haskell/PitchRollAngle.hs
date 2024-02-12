{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module PitchRollAngle where

import Language.Copilot
import Copilot.Compile.C99

import Utils

-------------------------------------------------------------------------------

runRollAnglePid :: Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float

runRollAnglePid kp ki dt integral_limit demand angle = kp * error + ki * integ

  where 

    error = demand - angle

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    integ' = [0] ++ integ

-------------------------------------------------------------------------------

runPitchAnglePid :: Stream Float -> 
                    Stream Float -> 
                    Stream Float -> 
                    Stream Float -> 
                    Stream Float -> 
                    Stream Float -> 
                    Stream Float

runPitchAnglePid kp ki dt integral_limit demand angle = kp * error + ki * integ

  where 

    error = demand - angle

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    integ' = [0] ++ integ

------------------------------------------------------------------------------

runPitchRollAnglePid :: (Stream Float, Stream Float) -> (Stream Float, Stream Float) ->
  (Stream Float, Stream Float)

runPitchRollAnglePid (rollDemand, pitchDemand) (rollAngle, pitchAngle) =
  (rollDemand', pitchDemand') 

  where kp = 6
        ki = 3
        dt = 0.01
        integral_limit = 20

        rollDemand'  = runRollAnglePid  kp ki dt integral_limit rollDemand  rollAngle
        pitchDemand' = runPitchAnglePid kp ki dt integral_limit pitchDemand pitchAngle
