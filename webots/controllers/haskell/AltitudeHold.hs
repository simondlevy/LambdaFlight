{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module AltitudeHold where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Clock

-- Utils ----------------------------------------------------------------------

rescale :: Stream Float -> Stream Float -> Stream Float -> Stream Float -> Stream Float
  -> Stream Float
rescale value oldmin oldmax newmin newmax =             
  (value - oldmin) / (oldmax - oldmin) * (newmax - newmin) + newmin

constrain :: Stream Float -> Stream Float -> Stream Float -> Stream Float
constrain val min max = if val < min then min else if val > max then max else val

