{--
  X/Y position PID control algorithm for real and simulated flight controllers
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

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

runXPid kp ki reset dt ilimit pitch dx thrust = pitch' where

  (pitch', integ) = piController kp ki dt ilimit pitch dx integ'

  zero = (thrust == 0) Language.Copilot.|| reset

  -- Reset error integral on zero thrust
  integ' = [0] ++ (if zero then 0 else integ)

------------------------------------------------------------------------------

runYPid kp ki reset dt ilimit roll dy thrust = roll' where

  (roll', integ) = piController kp ki dt ilimit roll dy integ'

  zero = (thrust == 0) Language.Copilot.|| reset

  -- Reset error integral on zero thrust
  integ' = [0] ++ (if zero then 0 else integ)

------------------------------------------------------------------------------

{--
  Demands are input as normalized interval [-1,+1] and output as angles in 
  degrees:

   roll:  input left positive => output negative

   pitch: input forward positive => output negative
--}

positionPid :: SBool -> ClosedLoopController

positionPid reset hover dt state demands = demands'  where

    kp = 25
    ki = 1
    ilimit = 5000
    
    angle = 30

    -- Rotate world-coordinate velocities into body coordinates
    psi' = deg2rad $ psi state
    cospsi = cos psi'
    sinpsi = sin psi'
    dx' = dx state
    dy' = dy state

    dxb =  dx'   * cospsi + dy' * sinpsi
    dyb = (-dx') * sinpsi + dy' * cospsi       

    thrust' = (thrust demands)
    roll' = (roll demands)
    pitch' = (pitch demands)

    -- Convert demand into angle, either by running a PI controller (hover mode)
    -- or just multiplying by a constant (non-hover mode)
    roll''   = if hover 
               then runYPid kp ki reset dt ilimit roll' dyb thrust'
               else roll' * 30

    pitch''  = if hover 
               then runXPid kp ki reset dt ilimit pitch' dxb  thrust'
               else pitch' * 30
 
    -- Negate roll, pitch demands on output
    demands' = Demands (thrust demands) (-roll'') (-pitch'') (yaw demands)
