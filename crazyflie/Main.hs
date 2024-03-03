{--
  LambdaFlight core algorithm for Crazyflie
 
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

module Main where

import Language.Copilot
import Copilot.Compile.C99

import Clock
import Core
import Motors

-- Scaling constants
clock_rate = RATE_500_HZ
tbase = 36000
tscale = 1000
tmin = 2000
prscale = 1
yscale = 1

spec = do

    let motors = realStep clock_rate tbase tscale tmin prscale yscale

    trigger "setMotors" true [
        arg $ Motors.qm1 motors, 
        arg $ Motors.qm2 motors, 
        arg $ Motors.qm3 motors, 
        arg $ Motors.qm4 motors] 

-- Compile the spec
main = reify spec >>= compileWith (CSettings "copilot_control_step" ".") "copilot_core"
