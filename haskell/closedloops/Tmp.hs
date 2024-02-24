{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Tmp where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

runTmp hover dt state demands = demands'  where

  thrustraw = thrust demands

  target = rescale thrustraw (-1) 1 0.2 2.0

  error = target - (z state)

  integ = if hover then integ' + error else 0

  integ' = [0] ++ integ

  thrustout = integ

  demands' = Demands thrustout 0 0 0
