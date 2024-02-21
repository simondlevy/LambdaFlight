{--
  Scaling constants for simulated flight controller
 
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

module Constants where

import Language.Copilot
import Copilot.Compile.C99

import Clock
import Utils

clock_rate = RATE_100_HZ

constants = ScalingConstants 48   -- thrust base 
                             0    -- thrust scale 
                             60   -- thrust min 
                             0.25 -- thrust max
                             30   -- pitch roll angle max
                             1e-4 -- pitch roll scale
                             4e-5 -- yaw scale

