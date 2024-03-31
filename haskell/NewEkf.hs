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

doinit :: SBool
doinit = extern "stream_doinit" Nothing

rawzero :: Array 2 (Array 2 Float)
rawzero =  array [
                array [0, 0], 
                array [0, 0]
               ] 

zero :: Stream (Array 2 (Array 2 Float))

zero = [ rawzero ] ++ zero

fun = False where

   pmat = if doinit then zero else pmat'

   pmat' = [rawzero] ++ pmat

spec = do


  -- let x = fun

  trigger "foo" true [arg $ true]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
