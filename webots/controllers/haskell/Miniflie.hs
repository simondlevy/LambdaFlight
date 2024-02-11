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

spec = do

  let thrust_base  = 48.0
  let thrust_scale = 0.25
  let thrust_min   = 0.0
  let thrust_max   = 60

  let dt = 0.01

  let state = liftState stateStruct

  let demands = liftDemands demandsStruct

  let thrust' = thrust demands

  -- let thrust'' = if inHoverMode then thrust' else thrust' * thrust_max

  let thrust'' = constrain (thrust' * thrust_scale + thrust_base) thrust_min thrust_max

  let demands'' = Demands thrust'' (roll demands) (pitch demands) (yaw demands)

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
