{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

-- import Prelude hiding ((>), (<), div, (++))

import Demands
import State
import Utils

-- PID controllers
import Altitude
import ClimbRate
import PitchRollRate
import YawAngle
import YawRate

-- Constants that will be different for sim vs. actual -----------------------

thrust_base  = 48.0
thrust_scale = 0.25
thrust_min   = 0.0
thrust_max   = 60

yaw_scale = 4e-5

pitch_roll_scale = 1e-4

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "demands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "state" Nothing

inHoverMode :: Stream Bool
inHoverMode = extern "in_hover_mode" Nothing

-- Main ----------------------------------------------------------------------

spec = do

  let state = liftState stateStruct

  let demands = liftDemands demandsStruct

  let thrust' = thrust demands

  let climbRate = runAltitudePid thrust' (z state)

  let thrust'' = if inHoverMode
                 then runClimbRatePid climbRate (dz state)
                 else thrust'

  let thrust''' = thrust'' * (if inHoverMode then 1 else thrust_max)

  let thrust'''' = constrain (thrust''' * thrust_scale + thrust_base)
                             thrust_min
                             thrust_max

  let yaw' = runYawAnglePid (yaw demands) (psi state)

  let yaw'' = runYawRatePid yaw' (dpsi state)

  trigger "setDemands" true [
                       arg $ thrust'''',
                       arg $ (roll demands) * pitch_roll_scale, 
                       arg $ (pitch demands) * pitch_roll_scale, 
                       arg $ yaw'' * yaw_scale
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
