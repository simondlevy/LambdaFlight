{--
  LambdaFlight core algorithm: reads open-loop demands and
  state as streams; runs PID controllers and motor mixers
 
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

module Core where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import Mixers
import Motors
import Utils

-- Streams from C++ ----------------------------------------------------------

-----------------------------------------------------------------------------

step = motors where

  motors = quadCFMixer $ Demands 0 0 0 0

------------------------------------------------------------------------------
 
spec = do

    let motors = step

    trigger "setMotors" true [
        arg $ Motors.qm1 motors, 
        arg $ Motors.qm2 motors, 
        arg $ Motors.qm3 motors, 
        arg $ Motors.qm4 motors] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
