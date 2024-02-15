module ClosedLoop where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import State
import Utils

type ClosedLoopController = SFloat -> State -> Demands -> Demands

piController kp ki dt ilimit target actual integ' = (output, integ) where

  error = target - actual

  integ = constrain (integ' + error * dt) (-ilimit) ilimit

  output = kp * error + ki * integ

 
pidController kp ki kd dt ilimit target actual error' integ' = 
  (output, error, integ) where

  error = target - actual

  integ = constrain (integ' + error * dt) (-ilimit) ilimit

  deriv = (error - error') / dt

  output = kp * error + ki * integ + kd * deriv
 
