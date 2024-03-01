{--
  Vehicle state datatype for real and simulated flight controllers

  From Eqn. (11) in Bouabdallah,  Murrieri, Siegwart (2004). 
  We use ENU coordinates based on 
  https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system
  Position in meters, velocity in meters/second, angles in degrees,
  angular velocity in degrees/second.

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

module State where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data StateStruct = StateStruct { 
    dx'     :: Field "dx" Float 
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
    dx     :: SFloat 
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

    toValues v = [ Value Float (dx' v)
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
                  )

liftState :: Stream StateStruct -> State
liftState state = State (state # dx') 
                        (state # dy') 
                        (state # z') 
                        (state # dz') 
                        (state # phi') 
                        (state # dphi') 
                        (state # theta') 
                        (state # dtheta') 
                        (state # psi') 
                        (state # dpsi') 
