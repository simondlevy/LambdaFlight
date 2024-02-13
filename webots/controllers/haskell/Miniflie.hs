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

  let thrust' = thrust demands

  let demands' = runAltitudePid state demands

  let thrust'' = if inHoverMode
                 then runClimbRatePid (thrust demands') (dz state)
                 else thrust'

  let (roll', pitch') = (roll demands, pitch demands)

  let (roll'', pitch'') = runPositionPid inHoverMode 
                                         (psi state) 
                                         (roll', pitch')
                                         (dx state, dy state)

  let demands'' = Demands thrust'' roll'' pitch'' (yaw demands')

  let demands''' = runPitchRollAnglePid state demands''

  let demands'''' = runPitchRollRatePid state demands'''

  let demands''''' = runYawAnglePid state demands''''

  let demands'''''' = runYawRatePid state demands'''''

  ----------------------------------------------------------------------------

  let thrust''' = thrust'' * (if inHoverMode then 1 else thrust_max)

  let thrust'''' = constrain (thrust''' * thrust_scale + thrust_base)
                             thrust_min
                             thrust_max

  let motors = quadCFMixer $ Demands thrust'''' 
                                     ((roll demands'''''') * pitch_roll_scale)
                                     ((pitch demands'''''') * pitch_roll_scale)
                                     ((yaw demands'''''') * yaw_scale)
  trigger "runMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
