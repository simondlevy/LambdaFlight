{--
  Miniflie algorithm for real and simulated flight controllers
 
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

import Demands
import Mixers
import Motors
import Scaling
import Utils

-- Constants -----------------------------------------------------------------

constants = ScalingConstants 36000 -- thrust base 
                             1000  -- thrust scale
                             20000 -- thrust min 
                             65535 -- thrust max 
                             1     -- pitch roll scale
                             1     -- yaw scale

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "finalDemands" Nothing

spec = do

  let demands = liftDemands demandsStruct

  let motors = quadCFMixer $ Demands (thrust demands) 
                                     ((roll demands) * (pitch_roll_scale constants))
                                     ((pitch demands) * (pitch_roll_scale constants))
                                     ((yaw demands) * (yaw_scale constants))

  trigger "setMotors" true [
                       arg $ qm1 motors, 
                       arg $ qm2 motors, 
                       arg $ qm3 motors, 
                       arg $ qm4 motors
                     ] 

-- Compile the spec
main = reify spec >>= compile "copilot"
