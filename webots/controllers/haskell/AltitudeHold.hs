{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module AltitudeHold where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Clock

-- Utils ----------------------------------------------------------------------

smax :: Stream Float -> Stream Float -> Stream Float
smax x y = if x > y then x else y

smin :: Stream Float -> Stream Float -> Stream Float
smin x y = if x < y then x else y

rescale :: Stream Float -> Stream Float -> Stream Float -> Stream Float -> Stream Float
  -> Stream Float
rescale value oldmin oldmax newmin newmax =             
  (value - oldmin) / (oldmax - oldmin) * (newmax - newmin) + newmin

constrain :: Stream Float -> Stream Float -> Stream Float -> Stream Float
constrain val min max = if val < min then min else if val > max then max else val

------------------------------------------------------------------------------
 
runAltitudeHold :: ClockRate -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float

runAltitudeHold updateRate thrust z dz = thrust
                
------------------------------------------------------------------------------

altitudeHold :: Stream Bool ->
                ClockRate -> 
                Stream Float -> 
                Stream Float -> 
                Stream Float -> 
                Stream Float

altitudeHold inHoverMode updateRate thrust z dz = thrust''

  where thrust_base  = 48.0
        thrust_scale = 0.25
        thrust_min   = 0.0
        thrust_max   = 60

        dt = rateToPeriod updateRate

        thrust' = if inHoverMode 
                  then runAltitudeHold updateRate thrust z dz
                  else thrust * thrust_max

        thrust'' = constrain (thrust' * thrust_scale + thrust_base)
                             thrust_min
                             thrust_max
