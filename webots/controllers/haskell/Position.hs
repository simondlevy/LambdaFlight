{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Position where

import Language.Copilot
import Copilot.Compile.C99

import Utils

type PosFun = SFloat -> SFloat -> SFloat

------------------------------------------------------------------------------

runXPid :: PosFun

runXPid pitchDemand dx = -(kp * error + ki * integ) -- note negation

  where 

    kp = 25
    ki = 1
    dt = 0.01

    error = pitchDemand - dx

    integ = integ' + error * dt

    integ' = [0] ++ integ

------------------------------------------------------------------------------

runYPid :: PosFun

runYPid rollDemand dy = -(kp * error + ki * integ) -- note negation

  where 

    kp = 25
    ki = 1
    dt = 0.01

    error = rollDemand - dy

    integ = integ' + error * dt

    integ' = [0] ++ integ

------------------------------------------------------------------------------

-- In non-hover mode, pitch/roll demands come in as [-1,+1], which we 
-- convert to degrees for input to pitch/roll controller

checkHoverMode :: SBool -> PosFun -> SFloat ->  SFloat ->  SFloat

checkHoverMode inHoverMode posFun rawDemand velocity = demand

  where demand = if inHoverMode 
                 then posFun rawDemand velocity
                 else rawDemand * 30

------------------------------------------------------------------------------

runPositionPid :: SBool ->
                  SFloat ->
                  (SFloat, SFloat) -> 
                  (SFloat, SFloat) ->
                  (SFloat, SFloat)

runPositionPid inHoverMode psi (rollDemand, pitchDemand) (dx, dy) =
  (rollDemand', pitchDemand') 

  where 

        -- Rotate world-coordinate velocities into body coordinates
        cospsi = cos psi
        sinpsi = sin psi
        dxb =  dx   * cospsi + dy * sinpsi
        dyb = (-dx) * sinpsi + dy * cospsi       

        rollDemand'  = checkHoverMode inHoverMode runYPid rollDemand dyb 

        pitchDemand'  = checkHoverMode inHoverMode runXPid pitchDemand dxb 
