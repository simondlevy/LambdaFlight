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

runRollAnglePid kp ki dt ilimit demand angle = demand'

  where 

    (demand', integ) = piController kp ki dt ilimit demand angle integ'

    integ' = [0] ++ integ

-------------------------------------------------------------------------------


pitchAnglePid kp ki dt ilimit demand angle = demand'

  where 

    (demand', integ) = piController kp ki dt ilimit demand angle integ'

    integ' = [0] ++ integ

------------------------------------------------------------------------------

pitchRollAnglePid :: ClosedLoopController

pitchRollAnglePid hover dt state demands = demands'

  where kp = 6
        ki = 3
        ilimit = 20

        roll'  = runRollAnglePid  kp ki dt ilimit (roll demands)  (phi state)
        pitch' = pitchAnglePid kp ki dt ilimit (pitch demands) (theta state)

        demands' = Demands (thrust demands) roll' pitch' (yaw demands)
