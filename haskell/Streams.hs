{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Streams where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data Foo = Foo { 
    x  :: SFloat 
  , y  :: SFloat 
  , z  :: SFloat 
}

spec = do

  let foo = Foo 0 0 0

  let bar = Foo 0 0 0

  let foo' = [bar] ++ foo

  trigger "foo" true [] 

-- Compile the spec
main = reify spec >>= 

  compileWith (CSettings "stream" ".") "stream"
