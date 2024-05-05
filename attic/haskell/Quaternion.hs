{--
  Quaternion support for LambdaFlight

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

module Quaternion where

import Language.Copilot hiding(atan2)

import Utils

data QuatStruct = QuatStruct { 
        qw' :: Field "w" Float 
      , qx' :: Field "x" Float 
      , qy' :: Field "y" Float 
      , qz' :: Field "z" Float 
}

data Quat = Quat { 
        qw  :: SFloat 
      , qx  :: SFloat 
      , qy  :: SFloat 
      , qz  :: SFloat 
}

instance Struct QuatStruct where

    typeName _ = "quat"

    toValues v = [  Value Float (qw' v)
                  , Value Float (qx' v)
                  , Value Float (qy' v)
                  , Value Float (qz' v)
                 ]

instance Typed QuatStruct where

  typeOf = Struct (QuatStruct
                   (Field 0) 
                   (Field 0) 
                   (Field 0) 
                   (Field 0) 
                  )

------------------------------------------------------------------------------

liftQuat :: Stream QuatStruct -> Quat

liftQuat quat = Quat (quat # qw') (quat # qx') (quat # qy') (quat # qz')

------------------------------------------------------------------------------

quat2euler :: Stream QuatStruct -> (SFloat, SFloat, SFloat)

quat2euler quat = (phi, theta, psi) where

  phi = atan2 (2 * (qy*qz + qw*qx)) (qw*qw - qx*qx - qy*qy + qz*qz)

  theta = asin ((-2) * (qx*qz - qw*qy))

  psi = atan2 (2 * (qx*qy + qw*qz)) (qw*qw + qx*qx - qy*qy - qz*qz) 

  qw = (quat # qw')
  qx = (quat # qx')
  qy = (quat # qy')
  qz = (quat # qz')
