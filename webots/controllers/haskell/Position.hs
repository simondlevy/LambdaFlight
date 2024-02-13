{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Position where

import Language.Copilot
import Copilot.Compile.C99

import ClosedLoop
import Demands
import State
import Utils

------------------------------------------------------------------------------

runXPid :: SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat

runXPid kp ki dt pitch' dy = -(kp * error + ki * integ) -- note negation

  where 

    error = pitch' - dy

    integ = integ' + error * dt

    integ' = [0] ++ integ

------------------------------------------------------------------------------

runYPid :: SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat

runYPid kp ki dt roll' dy = -(kp * error + ki * integ) -- note negation

  where 

    error = roll' - dy

    integ = integ' + error * dt

    integ' = [0] ++ integ

------------------------------------------------------------------------------

{--
  Demands are input as normalized interval [-1,+1] and output as angles in 
  degrees:

   roll:  input left positive => output negative

   pitch: input forward positive => output negative
--}

runPositionPid :: ClosedLoopController

runPositionPid state demands = demands'  where

    kp = 25
    ki = 1
    dt = 0.01

    -- Rotate world-coordinate velocities into body coordinates
    psi' = deg2rad $ psi state
    cospsi = cos psi'
    sinpsi = sin psi'
    dx' = dx state
    dy' = dy state

    dxb =  dx'   * cospsi + dy' * sinpsi
    dyb = (-dx') * sinpsi + dy' * cospsi       

    roll' = (roll demands)
    pitch' = (pitch demands)

    roll''  = runYPid kp ki dt roll'  dyb
    pitch'' = runXPid kp ki dt pitch' dxb
 
    demands' = Demands (thrust demands) roll'' pitch'' (yaw demands)
