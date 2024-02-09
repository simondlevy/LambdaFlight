{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module AltitudeHold where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Clock

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
 

altitudePid :: Stream Float -> Stream Float -> Stream Float -> Stream Float

altitudePid dt desired measured = output

  where kp = 2.0
        ki = 0.5
        integralLimit = 5000

        output_limit = 1.1

        error = desired - measured

        errorIntegral = smax (errorIntegral' + error) integralLimit

        errorIntegral' = [0] ++ errorIntegral

        output = smin (kp * error + ki * errorIntegral) output_limit

climbRatePid dt desired measured = kp * error + ki * errorIntegral

  where kp = 25
        ki = 15
        integralLimit = 5000

        error = desired - measured

        errorIntegral = smax (errorIntegral' + error) integralLimit

        errorIntegral' = [0] ++ errorIntegral

runAltitudeHold :: ClockRate -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float -> 
                   Stream Float

runAltitudeHold updateRate thrust z dz = thrust''

  where 

        thrust'' = thrust
        
{--
        -- In hover mode, thrust demand comes in as [-1,+1], so
        -- we convert it to a target altitude in meters
        thrust' = rescale thrust (-1) 1 0.2 2.0
 
        dt = rateToPeriod updateRate

        climbRate = altitudePid dt thrust' z

        thrust'' = climbRatePid dt climbRate dz
--}
                
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

        thrust' = if inHoverMode 
                  then runAltitudeHold updateRate thrust z dz
                  else thrust * thrust_max

        thrust'' = constrain (thrust' * thrust_scale + thrust_base)
                             thrust_min
                             thrust_max

 
