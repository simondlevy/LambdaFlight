{--
  LambdaFlight for simulated Crazyflie
 
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

module Constants where

import Utils
import Clock

clock_rate :: ClockRate
clock_rate = RATE_500_HZ

tbase :: SFloat
tbase = 36000

tscale :: SFloat
tscale = 1000

tmin :: SFloat
tmin = 2000

prscale :: SFloat
prscale = 1

yscale :: SFloat
yscale = 1
