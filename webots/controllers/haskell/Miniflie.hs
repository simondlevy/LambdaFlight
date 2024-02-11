{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

-- import Prelude hiding ((>), (<), div, (++))

import AltitudeHold
import Clock
import Demands
import Mixers
import Motors
import State

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "demands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "state" Nothing

inHoverMode :: Stream Bool
inHoverMode = extern "in_hover_mode" Nothing

thrust_base :: Stream Float
thrust_base = extern "thrust_base" Nothing

thrust_scale :: Stream Float
thrust_scale = extern "thrust_scale" Nothing

thrust_min :: Stream Float
thrust_min = extern "thrust_min" Nothing

thrust_max :: Stream Float
thrust_max = extern "thrust_max" Nothing

spec = do

  let state = liftState stateStruct

  let demands = liftDemands demandsStruct

  let thrust' = thrust demands

  let thrust'' = if inHoverMode
                 then runAltitudeHold (z state) (dz state) thrust'
                 else thrust'

  let thrust''' = thrust'' * (if inHoverMode then 1 else thrust_max)

  let thrust'''' = constrain (thrust''' * thrust_scale + thrust_base)
                             thrust_min
                             thrust_max

  let demands'' = Demands thrust'''' (roll demands) (pitch demands) (yaw demands)

  trigger "setDemands" true [
                       arg $ thrust demands'', 
                       arg $ roll demands'', 
                       arg $ pitch demands'', 
                       arg $ yaw demands''
                     ] 

{--
  let motors = quadCFMixer demands''

  trigger "runMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 
--}

-- Compile the spec
main = reify spec >>= compile "copilot"
