{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Demands where

import Language.Copilot
import Copilot.Compile.C99

data DemandsStruct = DemandsStruct { 
    thrust' :: Field "thrust" Float 
  , roll'   :: Field "roll" Float 
  , pitch'  :: Field "pitch" Float 
  , yaw'    :: Field "yaw" Float 
}

data Demands = Demands {

     thrust :: Stream Float
   , roll   :: Stream Float
   , pitch  :: Stream Float
   , yaw    :: Stream Float
}

liftDemands :: Stream DemandsStruct -> Demands
liftDemands demands = Demands (demands # thrust') 
                              (demands # roll') 
                              (demands # pitch') 
                              (demands # yaw') 

instance Struct DemandsStruct where

    typename _ = "demands" -- Name of the type in C

    toValues v = [ Value Float (thrust' v)
                 , Value Float (roll' v)
                 , Value Float (pitch' v)
                 , Value Float (yaw' v)
                 ]

instance Typed DemandsStruct where

  typeOf = Struct (DemandsStruct (Field 0) (Field 0) (Field 0) (Field 0))

getDemands :: Demands -> 
                (Stream Float, Stream Float, Stream Float, Stream Float)

getDemands demands = (thrust demands, roll demands, pitch demands, yaw demands)
