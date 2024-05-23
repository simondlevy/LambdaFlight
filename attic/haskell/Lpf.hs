{--
  Low-Pass Fitler functions
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Lpf where

import Language.Copilot
import Copilot.Compile.C99

import Utils

lpf :: SFloat -> SFloat -> SFloat -> SFloat

lpf sample_freq cutoff_freq sample = output where

  fr = sample_freq / cutoff_freq
  ohm = tan $ pi /fr
  c = 1 + 2 * cos (pi / 4) * ohm + ohm*ohm

  b0 = ohm * ohm / c
  b1 = 2 * b0
  b2 = b0
  a1 = 2 * (ohm * ohm - 1) / c
  a2 = (1 - 2 * cos (pi / 4) * ohm + ohm * ohm) / c

  delay_element_1 = 0
  delay_element_2 = 0

  delay_element_0 = sample - delay_element_1 * a1 - delay_element_2 * a2

  output = sample


