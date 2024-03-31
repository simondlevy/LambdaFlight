{--
  Linear ALgebra functions
 
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

module LinAlg where

-- import Data.Array

import Language.Copilot hiding(Array)
import Copilot.Compile.C99

-- import Utils

mmult :: (Ix i, Num a) => Array (i,i) a -> Array (i,i) a -> Array (i,i) a 

mmult x y 
  | x1 /= y0 || x1' /= y0'  = error "range mismatch"
  | otherwise               = array ((x0,y1),(x0',y1')) l
  where
    ((x0,x1),(x0',x1')) = bounds x
    ((y0,y1),(y0',y1')) = bounds y
    ir = range (x0,x0')
    jr = range (y1,y1')
    kr = range (x1,x1')
    l  = [((i,j), sum [x!(i,k) * y!(k,j) | k <- kr]) | i <- ir, j <- jr]

spec = do

    trigger "dummy" true [ ]

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_ekf" ".") "copilot_ekf"

