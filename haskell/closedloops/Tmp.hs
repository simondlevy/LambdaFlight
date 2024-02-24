{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Tmp where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

runTmp hover dt state demands = demands'  where

  reset = not hover

  cnt = if reset then 0 else z + 1

  z = [0] ++ cnt

  demands' = Demands cnt 0 0 0
