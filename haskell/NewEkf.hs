{--
  EKF algorithm
 
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

{-# LANGUAGE DataKinds        #-}
{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

import Utils

------------------------------------------------------------------------------

type EkfMode = SInt8

ekfMode :: EkfMode
ekfMode = extern "stream_ekfMode" Nothing

nowMsec :: SInt32
nowMsec = extern "stream_nowMsec" Nothing

mode_init      = 0 :: EkfMode
mode_predict   = 1 :: EkfMode
mode_update    = 7 :: EkfMode
mode_finalize  = 3 :: EkfMode
mode_get_state = 4 :: EkfMode

------------------------------------------------------------------------------

rawzero :: Array 7 (Array 7 Float)
rawzero =  array [
                array [0, 0, 0, 0, 0, 0, 0], 
                array [0, 0, 0, 0, 0, 0, 0], 
                array [0, 0, 0, 0, 0, 0, 0], 
                array [0, 0, 0, 0, 0, 0, 0], 
                array [0, 0, 0, 0, 0, 0, 0], 
                array [0, 0, 0, 0, 0, 0, 0], 
                array [0, 0, 0, 0, 0, 0, 0]
               ] 

zero :: Stream (Array 7 (Array 7 Float))

zero = [ rawzero ] ++ zero

fun = False where

   pmat = if ekfMode == mode_init then zero else pmat'

   pmat' = [rawzero] ++ pmat

spec = do

  -- let x = fun

  trigger "foo" true [arg $ true]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
