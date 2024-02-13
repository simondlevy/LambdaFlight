{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module PitchRollAngle where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

-------------------------------------------------------------------------------

runRollAnglePid :: SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat -> 
                   SFloat

runRollAnglePid kp ki dt integral_limit demand angle = kp * error + ki * integ

  where 

    error = demand - angle

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    integ' = [0] ++ integ

-------------------------------------------------------------------------------

runPitchAnglePid :: SFloat -> 
                    SFloat -> 
                    SFloat -> 
                    SFloat -> 
                    SFloat -> 
                    SFloat -> 
                    SFloat

runPitchAnglePid kp ki dt integral_limit demand angle = kp * error + ki * integ

  where 

    error = demand - angle

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    integ' = [0] ++ integ

------------------------------------------------------------------------------

runPitchRollAnglePid :: (SFloat, SFloat) -> (SFloat, SFloat) ->
  (SFloat, SFloat)

runPitchRollAnglePid (rollDemand, pitchDemand) (rollAngle, pitchAngle) =
  (rollDemand', pitchDemand') 

  where kp = 6
        ki = 3
        dt = 0.01
        integral_limit = 20

        rollDemand'  = runRollAnglePid  kp ki dt integral_limit rollDemand  rollAngle
        pitchDemand' = runPitchAnglePid kp ki dt integral_limit pitchDemand pitchAngle

------------------------------------------------------------------------------
------------------------------------------------------------------------------

newRunPitchRollAnglePid :: ClosedLoopController

newRunPitchRollAnglePid inHoverMode state demands = demands'

  where kp = 6
        ki = 3
        dt = 0.01
        integral_limit = 20

        demands' = demands
