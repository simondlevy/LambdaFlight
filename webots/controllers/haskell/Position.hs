{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Position where

import Language.Copilot
import Copilot.Compile.C99

import Utils

------------------------------------------------------------------------------

runXPid :: SFloat -> SFloat -> SFloat

runXPid pitchDemand dx = -(kp * error + ki * integ) -- note negation

  where 

    kp = 25
    ki = 1
    dt = 0.01

    error = pitchDemand - dx

    integ = integ' + error * dt

    integ' = [0] ++ integ

------------------------------------------------------------------------------

runYPid :: SFloat -> SFloat -> SFloat

runYPid rollDemand dy = -(kp * error + ki * integ) -- note negation

  where 

    kp = 25
    ki = 1
    dt = 0.01

    error = rollDemand - dy

    integ = integ' + error * dt

    integ' = [0] ++ integ

------------------------------------------------------------------------------

{--
  Demands are input as normalized interval [-1,+1] and output as angles in 
  degrees:

   roll:  input left positive => output negative

   pitch: input forward positive => output negative
--}

runPositionPid :: SBool ->
                  SFloat ->
                  (SFloat, SFloat) -> 
                  (SFloat, SFloat) ->
                  (SFloat, SFloat)

runPositionPid inHoverMode psi (rollDemand, pitchDemand) (dx, dy) =
  (rollDemand', pitchDemand') 

  where 

        -- Rotate world-coordinate velocities into body coordinates
        dpsi = deg2rad psi
        cospsi = cos dpsi
        sinpsi = sin dpsi
        dxb =  dx   * cospsi + dy * sinpsi
        dyb = (-dx) * sinpsi + dy * cospsi       

        -- In hover mode, pitch/roll demands comes in as meters/second, which
        -- we convert to degrees for feeding to the angle controller.  In 
        -- non-hover mode, we convert the demands directly to angles in 
        -- [-30,+30].
        rollDemand' = if inHoverMode then runYPid rollDemand dyb else 30 * rollDemand
        pitchDemand' = if inHoverMode then runXPid pitchDemand dxb else 30 * pitchDemand

