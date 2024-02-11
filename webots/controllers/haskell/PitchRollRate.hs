{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module PitchRollRate where

import Language.Copilot
import Copilot.Compile.C99

runRollRatePid :: Stream Float -> 
                  Stream Float -> 
                  Stream Float -> 
                  Stream Float -> 
                  Stream Float -> 
                  Stream Float -> 
                  Stream Float

runRollRatePid kp ki kd integralLimit demand rate = 0

------------------------------------------------------------------------------

runPitchRatePid :: Stream Float -> 
                  Stream Float -> 
                  Stream Float -> 
                  Stream Float -> 
                  Stream Float -> 
                  Stream Float -> 
                  Stream Float

runPitchRatePid kp ki kd integralLimit demand rate = 0

------------------------------------------------------------------------------

runPitchRollRatePid :: (Stream Float, Stream Float) -> (Stream Float, Stream Float) ->
  (Stream Float, Stream Float)

runPitchRollRatePid (rollDemand, pitchDemand) (rollRate, pitchRate) =
  (rollDemand', pitchDemand') 

  where kp = 125
        ki = 250
        kd = 1.25
        integral_limit = 33

        rollDemand'  = runRollRatePid  kp ki kd integral_limit rollDemand  rollRate
        pitchDemand' = runPitchRatePid kp ki kd integral_limit pitchDemand pitchRate
