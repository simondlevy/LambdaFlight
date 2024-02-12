{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Position where

import Language.Copilot
import Copilot.Compile.C99

import Utils

------------------------------------------------------------------------------

runYPid :: SFloat -> 
           SFloat -> 
           SFloat -> 
           SBool -> 
           SFloat -> 
           SFloat -> 
           SFloat -> 
           SFloat

runYPid kp ki dt inHoverMode integral_limit rollDemand dy = 0

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
        integral_limit = 20

        -- Rotate world-coordinate velocities into body coordinates
        cospsi = cos psi
        sinpsi = sin psi
        dxb =  dx   * cospsi + dy * sinpsi
        dyb = (-dx) * sinpsi + dy * cospsi       

        rollDemand'  = runYPid kp ki dt inHoverMode integral_limit rollDemand dyb 
        pitchDemand' = 0
