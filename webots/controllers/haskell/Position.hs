{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Position where

import Language.Copilot
import Copilot.Compile.C99

import Utils

type PosFun = SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat

------------------------------------------------------------------------------

runXPid :: PosFun

runXPid kp ki dt output_limit pitchDemand dx = pitchDemand'

  where 

    -- note negation
    pitchDemand' = constrain (-(kp * error + ki * integ))
                             (-output_limit) 
                             output_limit

    error = pitchDemand - dx

    integ = integ' + error * dt

    integ' = [0] ++ integ

------------------------------------------------------------------------------

runYPid :: PosFun

runYPid kp ki dt output_limit rollDemand dy = rollDemand'

  where 

    -- note negation
    rollDemand' = constrain (-(kp * error + ki * integ))
                            (-output_limit) 
                            output_limit

    error = rollDemand - dy

    integ = integ' + error * dt

    integ' = [0] ++ integ

------------------------------------------------------------------------------

-- In non-hover mode, pitch/roll demands come in as [-1,+1], which we 
-- convert to degrees for input to pitch/roll controller

checkHoverMode :: SBool ->
                  PosFun -> 
                  SFloat ->  
                  SFloat ->  
                  SFloat ->  
                  SFloat ->  
                  SFloat ->  
                  SFloat ->  
                  SFloat

checkHoverMode inHoverMode 
               posFun 
               kp 
               ki 
               dt 
               output_limit 
               rawDemand 
               velocity = demand

  where demand = if inHoverMode 
                 then posFun kp ki dt output_limit rawDemand velocity
                 else rawDemand * 30

------------------------------------------------------------------------------

runPositionPid :: SBool ->
                  SFloat ->
                  (SFloat, SFloat) -> 
                  (SFloat, SFloat) ->
                  (SFloat, SFloat)

runPositionPid inHoverMode psi (rollDemand, pitchDemand) (dx, dy) =
  (rollDemand', pitchDemand') 

  where kp = 25
        ki = 1
        dt = 0.01
        output_limit = 22

        -- Rotate world-coordinate velocities into body coordinates
        cospsi = cos psi
        sinpsi = sin psi
        dxb =  dx   * cospsi + dy * sinpsi
        dyb = (-dx) * sinpsi + dy * cospsi       

        rollDemand'  = checkHoverMode inHoverMode 
                                      runYPid 
                                      kp 
                                      ki 
                                      dt 
                                      output_limit 
                                      rollDemand 
                                      dyb 

        pitchDemand'  = checkHoverMode inHoverMode 
                                       runXPid 
                                       kp 
                                       ki 
                                       dt 
                                       output_limit 
                                       pitchDemand 
                                       dxb 
