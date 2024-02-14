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

pitchAnglePid :: SFloat -> 
                    SFloat -> 
                    SFloat -> 
                    SFloat -> 
                    SFloat -> 
                    SFloat -> 
                    SFloat

pitchAnglePid kp ki dt integral_limit demand angle = kp * error + ki * integ

  where 

    error = demand - angle

    integ = constrain (integ' + error * dt) (-integral_limit) integral_limit

    integ' = [0] ++ integ

------------------------------------------------------------------------------

pitchRollAnglePid :: ClosedLoopController

pitchRollAnglePid dt state demands = demands'

  where kp = 6
        ki = 3
        integral_limit = 20

        roll'  = runRollAnglePid  kp ki dt integral_limit (roll demands)  (phi state)
        pitch' = pitchAnglePid kp ki dt integral_limit (pitch demands) (theta state)

        demands' = Demands (thrust demands) roll' pitch' (yaw demands)
