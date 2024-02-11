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

altitudeHold inHoverMode updateRate thrust z dz = thrust''

  where thrust_base  = 48.0
        thrust_scale = 0.25
        thrust_min   = 0.0
        thrust_max   = 60

        dt = rateToPeriod updateRate

        thrust' = if inHoverMode then thrust else thrust * thrust_max

        thrust'' = constrain (thrust' * thrust_scale + thrust_base)
                             thrust_min
                             thrust_max
