{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Tmp where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

runTmp hover dt state demands = demands'  where

  ki = 0.5
  ilimit = 5000

  thrustraw = thrust demands

  target = rescale thrustraw (-1) 1 0.2 2.0

  error = target - (z state)

  integ = if hover 
          then constrain (integ' + error) (-ilimit) ilimit
          else 0

  integ' = [0] ++ integ

  thrustout = if hover then ki * integ else thrustraw

  demands' = Demands thrustout 0 0 0
