{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Utils where

import Language.Copilot
import Copilot.Compile.C99

type SFloat = Stream Float
type SBool = Stream Bool

rescale :: SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat
rescale value oldmin oldmax newmin newmax =             
  (value - oldmin) / (oldmax - oldmin) * (newmax - newmin) + newmin

constrain :: SFloat -> SFloat -> SFloat -> SFloat
constrain val min max = if val < min then min else if val > max then max else val

rad2deg :: SFloat -> SFloat
rad2deg rad = 180 * rad / 3.1415928
