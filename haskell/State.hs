{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module State where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data StateStruct = StateStruct { 
    x'      :: Field "x" Float 
  , dx'     :: Field "dx" Float 
  , y'      :: Field "y" Float 
  , dy'     :: Field "dy" Float 
  , z'      :: Field "z" Float 
  , dz'     :: Field "dz" Float 
  , phi'    :: Field "phi" Float 
  , dphi'   :: Field "dphi" Float 
  , theta'  :: Field "theta" Float 
  , dtheta' :: Field "dtheta" Float 
  , psi'    :: Field "psi" Float 
  , dpsi'   :: Field "dpsi" Float 
}

data State = State { 
    x      :: SFloat 
  , dx     :: SFloat 
  , y      :: SFloat 
  , dy     :: SFloat 
  , z      :: SFloat 
  , dz     :: SFloat 
  , phi    :: SFloat 
  , dphi   :: SFloat 
  , theta  :: SFloat 
  , dtheta :: SFloat 
  , psi    :: SFloat 
  , dpsi   :: SFloat 
}

instance Struct StateStruct where

    typename _ = "state" -- Name of the type in C

    toValues v = [ Value Float (x' v)
                 , Value Float (dx' v)
                 , Value Float (y' v)
                 , Value Float (dy' v)
                 , Value Float (z' v)
                 , Value Float (dz' v)
                 , Value Float (phi' v)
                 , Value Float (dphi' v)
                 , Value Float (theta' v)
                 , Value Float (dtheta' v)
                 , Value Float (psi' v)
                 , Value Float (dpsi' v)
                 ]

instance Typed StateStruct where

  typeOf = Struct (StateStruct
                   (Field 0) 
                   (Field 0) 
                   (Field 0) 
                   (Field 0)
                   (Field 0)
                   (Field 0)
                   (Field 0)
                   (Field 0)
                   (Field 0)
                   (Field 0)
                   (Field 0)
                   (Field 0)
                  )

liftState :: Stream StateStruct -> State
liftState state = State (state # x') 
                        (state # dx') 
                        (state # y') 
                        (state # dy') 
                        (state # z') 
                        (state # dz') 
                        (state # phi') 
                        (state # dphi') 
                        (state # theta') 
                        (state # dtheta') 
                        (state # psi') 
                        (state # dpsi') 
