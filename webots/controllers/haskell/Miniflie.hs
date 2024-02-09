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

  let state = liftState stateStruct

  let demands' = liftDemands demandsStruct

  let thrust' = altitudeHold inHoverMode RATE_100_HZ (thrust demands') (z state) (dz state)

  let demands'' = Demands thrust' (roll demands') (pitch demands') (yaw demands')

  let motors = quadCFMixer demands''

  trigger "runMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
