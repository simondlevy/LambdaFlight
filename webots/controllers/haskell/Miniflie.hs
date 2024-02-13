{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

-- import Prelude hiding ((>), (<), div, (++))

import Demands
import Mixers
import Motors
import State
import Utils

-- PID controllers
import Altitude
import ClimbRate
import PitchRollAngle
import PitchRollRate
import Position
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

inHoverMode :: SBool
inHoverMode = extern "in_hover_mode" Nothing

-- Main ----------------------------------------------------------------------

spec = do

  let state = liftState stateStruct

  let demands = liftDemands demandsStruct

  ----------------------------------------------------------------------------

  let thrust1 = thrust demands

  let demands1 = runAltitudePid state demands

  let thrust2 = if inHoverMode
                 then runClimbRatePid (thrust demands1) (dz state)
                 else thrust1

  let (roll1, pitch1) = (roll demands, pitch demands)

  let (roll2, pitch2) = runPositionPid inHoverMode 
                                         (psi state) 
                                         (roll1, pitch1)
                                         (dx state, dy state)

  let demands2 = Demands thrust2 roll2 pitch2 (yaw demands1)

  let demands3 = runPitchRollAnglePid state demands2

  let demands4 = runPitchRollRatePid state demands3

  let demands5 = runYawAnglePid state demands4

  let demands6 = runYawRatePid state demands5

  ----------------------------------------------------------------------------

  let thrust3 = thrust2 * (if inHoverMode then 1 else thrust_max)

  let thrust4 = constrain (thrust3 * thrust_scale + thrust_base)
                             thrust_min
                             thrust_max

  let motors = quadCFMixer $ Demands thrust4 
                                     ((roll demands6) * pitch_roll_scale)
                                     ((pitch demands6) * pitch_roll_scale)
                                     ((yaw demands6) * yaw_scale)
  trigger "runMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
