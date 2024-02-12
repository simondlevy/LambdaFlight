{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Motors where

import Language.Copilot
import Copilot.Compile.C99

data Motors = QuadMotors { 
                       qm1 :: SFloat
                     , qm2 :: SFloat  
                     , qm3 :: SFloat  
                     , qm4 :: SFloat   
               } |

              HexMotors {
                    hm1 :: SFloat  
                  , hm2 :: SFloat  
                  , hm3 :: SFloat  
                  , hm4 :: SFloat  
                  , hm5 :: SFloat  
                  , hm6 :: SFloat  
               } deriving (Show)
