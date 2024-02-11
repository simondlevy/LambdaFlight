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

------------------------------------------------------------------------------

altitudePid :: Stream Float -> Stream Float -> Stream Float
altitudePid desired measured = kp * error + ki * integ

  where

    kp = 2
    ki = 0.5
    dt = 0.01

    integration_limit = 5000

    error = desired - measured

    integ = constrain (integ' + error * dt) (-integration_limit) integration_limit

    integ' = [0] ++ integ

------------------------------------------------------------------------------

climbRatePid :: Stream Float -> Stream Float -> Stream Float
climbRatePid desired measured = kp * error + ki * integ

  where

    kp = 25
    ki = 15
    dt = 0.01

    integration_limit = 5000

    error = desired - measured

    integ = constrain (integ' + error * dt) (-integration_limit) integration_limit

    integ' = [0] ++ integ

------------------------------------------------------------------------------

runAltitudeHold :: Stream Float -> Stream Float -> Stream Float -> Stream Float
runAltitudeHold z dz thrust = thrust'

  where

    -- In hover mode, thrust demand comes in as [-1,+1], so
    -- we convert it to a target altitude in meters
    sthrust = rescale thrust (-1) 1 0.2 2.0

    -- Set climb rate based on target altitude
    climbRate = altitudePid sthrust z 

    -- Set thrust for desired climb rate
    thrust' = climbRatePid climbRate dz 


