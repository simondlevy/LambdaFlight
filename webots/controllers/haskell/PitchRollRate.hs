{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module PitchRollRate where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

-------------------------------------------------------------------------------

runRollRatePid kp ki kd dt ilimit demand rate = demand'

  where 

    (demand', error, integ) = pidController kp ki kd dt ilimit demand rate error' integ'

    integ' = [0] ++ integ
    error' = [0] ++ error

-------------------------------------------------------------------------------

pitchRatePid kp ki kd dt ilimit demand rate = demand'

  where 

    (demand', error, integ) = pidController kp ki kd dt ilimit demand rate error' integ'


    integ' = [0] ++ integ
    error' = [0] ++ error

------------------------------------------------------------------------------

pitchRollRatePid :: ClosedLoopController

pitchRollRatePid dt state demands = demands' where

  kp = 125
  ki = 250
  kd = 1.25
  ilimit = 33

  roll'  = runRollRatePid  kp ki kd dt ilimit (roll demands) (dphi state)
  pitch' = pitchRatePid kp ki kd dt ilimit (pitch demands) (dtheta state)

  demands' = Demands (thrust demands) roll' pitch' (yaw demands)


