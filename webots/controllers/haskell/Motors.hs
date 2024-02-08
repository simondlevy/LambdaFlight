{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Motors where

import Language.Copilot
import Copilot.Compile.C99

data Motors = QuadMotors { 
                       qm1 :: Stream Float
                     , qm2 :: Stream Float  
                     , qm3 :: Stream Float  
                     , qm4 :: Stream Float   
               } |

              HexMotors {
                    hm1 :: Stream Float  
                  , hm2 :: Stream Float  
                  , hm3 :: Stream Float  
                  , hm4 :: Stream Float  
                  , hm5 :: Stream Float  
                  , hm6 :: Stream Float  
               } deriving (Show)
