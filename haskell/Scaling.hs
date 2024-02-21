{--
  Clock rates for real and simulated flight controllers
 
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

module Scaling where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data ScalingConstants = ScalingConstants {

    thrust_base :: SFloat
  , thrust_scale :: SFloat
  , thrust_min :: SFloat
  , thrust_max :: SFloat
  , pitch_roll_angle_max :: SFloat
  , pitch_roll_scale :: SFloat
  , yaw_scale :: SFloat
}
