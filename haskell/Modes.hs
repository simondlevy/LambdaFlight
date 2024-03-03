{--
  Modes for real flight controllers
 
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

module Modes where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data Mode =
         MODE_RUN_CORE
       | MODE_KALMAN_INIT
       | MODE_KALMAN_PREDICT
       | MODE_KALMAN_UPDATE
       | MODE_KALMAN_FINALIZE
       | MODE_KALMAN_GET_STATE

modeToInt :: Mode -> SInt8
modeToInt MODE_RUN_CORE         = 0
modeToInt MODE_KALMAN_INIT      = 1
modeToInt MODE_KALMAN_PREDICT   = 2
modeToInt MODE_KALMAN_UPDATE    = 3
modeToInt MODE_KALMAN_FINALIZE  = 4
modeToInt MODE_KALMAN_GET_STATE = 5


