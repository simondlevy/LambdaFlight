{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Tmp where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

runTmp hover dt state demands = demands'  where

  cnt = if hover then z + 1 else 0

  z = [0] ++ cnt

  demands' = Demands cnt 0 0 0
