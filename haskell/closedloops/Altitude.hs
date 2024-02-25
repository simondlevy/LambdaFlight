{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Altitude where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

altitudePid hover dt state demands = demands'  where

  kp = 2.0
  ki = 0.5
  ilimit = 5000

  thrustraw = thrust demands

  target = rescale thrustraw (-1) 1 0.2 2.0

  error = target - (z state)

  integ = if hover 
          then constrain (integ' + error * dt) (-ilimit) ilimit
          else 0

  integ' = [0] ++ integ

  thrustout = if hover then kp * error + ki * integ else thrustraw

  demands' = Demands thrustout (roll demands) (pitch demands) (yaw demands)
