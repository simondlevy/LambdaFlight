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

pitch_roll_angle_max = 30

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

  let pids = [altitudePid,
              climbRatePid,
              (positionPid inHoverMode pitch_roll_angle_max),
              pitchRollAnglePid, 
              pitchRollRatePid, 
              yawAnglePid, 
              yawRatePid]

  let demands' = foldl (\d f -> f state d) demands pids

  ----------------------------------------------------------------------------

  -- In hover mode, we scale the thrust so as to keep the vehicle level; 
  -- otherwise, we just scale it by its maximum value
  let thrust'' = if inHoverMode
                 then constrain ((thrust demands') * thrust_scale + thrust_base)
                                 thrust_min thrust_max
                 else (thrust demands) * thrust_max

  let motors = quadCFMixer $ Demands thrust'' 
                                     ((roll demands') * pitch_roll_scale)
                                     ((pitch demands') * pitch_roll_scale)
                                     ((yaw demands') * yaw_scale)
  trigger "runMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
