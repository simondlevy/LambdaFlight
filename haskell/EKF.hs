{--
  EKF algorithm for Crazyflie
 
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

import State
import Utils

type EkfMode = SInt8

ekfMode :: EkfMode
ekfMode = extern "stream_ekfMode" Nothing

mode_init      = 0 :: EkfMode
mode_predict   = 1 :: EkfMode
mode_update    = 2 :: EkfMode
mode_finalize  = 3 :: EkfMode
mode_get_state = 4 :: EkfMode

data Ekf = Ekf { 
                   qw :: SFloat 
                 , qx :: SFloat 
                 , qy :: SFloat 
                 , qz :: SFloat 

                 , lastPredictionMsec :: SInt32
               }

runEkf :: SInt32 -> Ekf

runEkf nowMsec = Ekf qw qx qy qz lastPredictionMsec

   where init = ekfMode == mode_init

         qw = if init then 1 else qw'
         qx = if init then 0 else qx'
         qy = if init then 0 else qw'
         qz = if init then 0 else qz'

         lastPredictionMsec = if init then nowMsec else lastPredictionMsec'

         qw' = [1] ++ qw
         qx' = [0] ++ qx
         qy' = [0] ++ qy
         qz' = [0] ++ qz

         lastPredictionMsec' = [0] ++ lastPredictionMsec



spec = do

    trigger "dummy" true [ ]

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_ekf" ".") "copilot_ekf"
