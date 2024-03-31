-- Copyright © 2019 National Institute of Aerospace / Galois, Inc.

-- | This is a simple example for arrays. As a program, it does not make much
-- sense, however it shows of the features of arrays nicely.

-- | Enable compiler extension for type-level data, necesary for the array
-- length.

{-# LANGUAGE DataKinds        #-}
{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

type SFloat = Stream Float

type SBool = Stream Bool

doinit :: SBool
doinit = extern "stream_doinit" Nothing

farr :: Stream (Array 2 (Array 2 Float))
farr = [ array [
                array [0, 0], 
                array [0, 0]
               ] 
       ] ++ farr

zero :: Stream (Array 2 (Array 2 Float))
zero = [ array [
                array [0, 0], 
                array [0, 0]
               ] 
       ] ++ zero

fun = False where

   foo = if doinit then zero else farr'

   farr' = [ array [
                array [0, 0], 
                array [0, 0]
               ] 
           ] ++ foo


spec = do


  -- let x = fun

  trigger "foo" true [arg $ true]

main = reify spec >>= 

  compileWith (CSettings "foo" ".") "foo"
