{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module PitchRollRate where

import Language.Copilot
import Copilot.Compile.C99

runPitchRollRatePid :: (Stream Float, Stream Float) -> (Stream Float, Stream Float) ->
  (Stream Float, Stream Float)

runPitchRollRatePid (rollDemand, pitchDemand) (rollRate, pitchRate) =
  (rollDemand', pitchDemand') 

  where rollDemand' = 0
        pitchDemand' = 0
