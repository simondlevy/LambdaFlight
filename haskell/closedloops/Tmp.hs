{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Tmp where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

runTmp hover dt state demands = demands'  where

  integ = if hover then integ' + 1 else 0

  integ' = [0] ++ integ

  demands' = Demands integ 0 0 0
